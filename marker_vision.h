#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <unistd.h> // for sleep
using namespace cv;
using namespace std;

//#define WAIT_KEY_MILL      1 // tiempo de espera entre fotogramas cuando se abre la ventana, si vale 0, solo avanza cuando se presiona alguna tecla
#define AUTO_SCALE_FACTOR 1

//#define ROT_POS_ORI

class VisionClass {
    public:
	VisionClass(){
        bool readOk = readVisionParameters("../vision_params.yml");
        if(!readOk) {
            cerr << "Invalid general vision parameters file" << endl;
            exit(0);
        }

        /*** Vision setup ***/
    	detectorParams = aruco::DetectorParameters::create();
        readOk = readDetectorParameters( "../detector_params.yml", detectorParams );
        if(!readOk) {
            cerr << "Invalid detector parameters file" << endl;
            exit(0);
        }

    	dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dict_type));

    	readOk = readCameraParameters(calibration_file, camMatrix, distCoeffs);
    	if(!readOk) {
    	    cerr << "Invalid camera file" << endl;
    	    exit(0);
    	}

        if (charuco){
            charucoboard = aruco::CharucoBoard::create(squaresX, squaresY, square_length, marker_length_ch, dictionary);
            board = charucoboard.staticCast<aruco::Board>();
            axisLength = 0.5f * ((float)min(squaresX, squaresY) * (square_length));
        }
        else{
            axisLength = 0.5f * marker_length;
        }


        if (video_file!=""){
    		inputVideo.open(video_file);
        }
        else{
    		inputVideo.open(0);
            inputVideo.set(CAP_PROP_FRAME_WIDTH, frame_width);
            inputVideo.set(CAP_PROP_FRAME_HEIGHT, frame_height);
            string cmd;
            if (fps!=0){
                cmd=(string)"v4l2-ctl -d /dev/video0 -p "+ to_string(fps);
                const char* aux1=cmd.data();
                system(aux1);
            }
            if (exposure_time!=0){
                cmd=(string)"v4l2-ctl -d /dev/video0 -c auto_exposure=1 -c exposure_time_absolute="+ to_string(exposure_time);
                const char* aux2=cmd.data();
                system(aux2);
            }
	        else{
		        system("v4l2-ctl -d /dev/video0 -c auto_exposure=0");
            }
            system(" v4l2-ctl -V");
        }
        /* Test an image */
        grab_and_retrieve_image();
        cout << "Ancho de la imagen:\t"<< image.cols << endl;
        cout << "Alto de la imagen:\t"<< image.rows << endl << endl;
	}

	int grab_and_retrieve_image(){
       	    int res = inputVideo.grab();
            inputVideo.retrieve(image);
            return res;
	}

	bool detect_marker(Eigen::Vector3d &pos, Eigen::Vector3d &eul);

    private: 	
        bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params);
        bool readVisionParameters(string filename);
        void InvertPose(Eigen::Vector3d &pos, Eigen::Vector3d &eul, Vec3d &rvec, Vec3d &tvec);
        bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs); 
        Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R);

        Mat image;
        Mat imageCopy;
    	VideoCapture inputVideo;
    	Ptr<aruco::Dictionary> dictionary;
        Ptr<aruco::DetectorParameters> detectorParams;
    	Mat camMatrix; 
        Mat distCoeffs;
        string calibration_file;
        string video_file;
        bool show_rejected;
        float marker_length;
        int dict_type;
        float axisLength;
        bool open_window;
        bool write_images;
        // camera config
        int exposure_time;
        int fps;
        int frame_width;
        int frame_height;
        // diamond specific
        bool diamond;
        bool autoScale;
        // charuco specific
        bool charuco;
        bool refindStrategy;
        int squaresX;
        int squaresY;
        float square_length;
        float marker_length_ch;
        Ptr<aruco::CharucoBoard> charucoboard;
        Ptr<aruco::Board> board;
        // logging
        int totalIterations=0;

};


// Calculates rotation matrix to euler angles
Eigen::Vector3d VisionClass::rotationMatrixToEulerAngles(Eigen::Matrix3d &R)
{

    float sy = sqrt(R(0,0) * R(0,0) +  R(1,0) * R(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R(2,1) , R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2(R(1,0), R(0,0));
    }
    else
    {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }
    return Eigen::Vector3d(x, y, z);
}


bool VisionClass::readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}


bool VisionClass::detect_marker(Eigen::Vector3d &pos, Eigen::Vector3d &eul){

    vector< int > ids, charucoIds;
    vector< vector< Point2f > > corners, rejected;
    vector< Vec3d > rvecs, tvecs;
    Vec3d rvec, tvec;
    vector< Point2f > charucoCorners;
    vector< vector< Point2f > > diamondCorners;
    vector< Vec4i > diamondIds;


    aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

    bool found_marker=ids.size() > 0;
    bool valid_pose = false;
    int interpolatedCorners = 0;

    if (charuco){
        if(refindStrategy)
             aruco::refineDetectedMarkers(image, board, corners, ids, rejected,
                                             camMatrix, distCoeffs);

        // interpolate charuco corners
        if(found_marker)
            interpolatedCorners =
                aruco::interpolateCornersCharuco(corners, ids, image, charucoboard,
                                                 charucoCorners, charucoIds, camMatrix, distCoeffs);
        if ((int)ids.size()==17){
            // estimate charuco board pose
            valid_pose = aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoboard,
                                                    camMatrix, distCoeffs, rvec, tvec);
        }
        else{
            valid_pose= false;
        }
    }
    else if (diamond){
        if (found_marker){
            aruco::detectCharucoDiamond(image, corners, ids,
                                        square_length / marker_length_ch, diamondCorners, diamondIds,
                                        camMatrix, distCoeffs);

            if(!autoScale) {
                aruco::estimatePoseSingleMarkers(diamondCorners, square_length, camMatrix,
                                                 distCoeffs, rvecs, tvecs);
            } else {
                // if autoscale, extract square size from last diamond id
                //for(unsigned int i = 0; i < diamondCorners.size(); i++) {
                //    float autoSquareLength = AUTO_SCALE_FACTOR * float(diamondIds[i].val[3]);
                //    vector< vector< Point2f > > currentCorners;
                //    vector< Vec3d > currentRvec, currentTvec;
                //    currentCorners.push_back(diamondCorners[i]);
                //    aruco::estimatePoseSingleMarkers(currentCorners, autoSquareLength, camMatrix,
                //                                     distCoeffs, currentRvec, currentTvec);
                //    rvecs.push_back(currentRvec[0]);
                //    tvecs.push_back(currentTvec[0]);
                //}
                cout << "Autoscale todavía no implementado" <<endl;
                exit(0);
            }
            if (tvecs.size()>0){
                rvec=rvecs[0];
                tvec=tvecs[0];
                valid_pose= true;
            }
        }
    }
    else{
        if(found_marker){
            aruco::estimatePoseSingleMarkers(corners, marker_length, camMatrix, distCoeffs, rvecs, tvecs);
            valid_pose = true;
            rvec=rvecs[0];
            tvec=tvecs[0];
        }
     }
        

     if (open_window){

        image.copyTo(imageCopy);

        if(found_marker){
           aruco::drawDetectedMarkers(imageCopy, corners, ids);
        }
        if(interpolatedCorners > 0) {
            Scalar color;
            color = Scalar(255, 0, 0);
            aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);
        }
        if (valid_pose){
           aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec, tvec, axisLength);
        }

        if(show_rejected && rejected.size() > 0)
            aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

        imshow("out", imageCopy);
        if (write_images){
            char path [100];
            sprintf(path,"../results/latest/images/image%d.png", totalIterations);
            imwrite(path,imageCopy);
        }
//        waitKey( WAIT_KEY_MILL );
     }

    if (valid_pose){
        InvertPose(pos, eul, rvec, tvec);
    }
    else{
        pos[0]=pos[1]=pos[2]=NAN;
        eul[0]=eul[1]=eul[2]=NAN;
    }
    
    totalIterations++;

    return valid_pose;
}

void VisionClass::InvertPose(Eigen::Vector3d &pos, Eigen::Vector3d &eul, Vec3d &rvec, Vec3d &tvec){
    /* @brief Invierte la posición y la rotación. También corrige la posición de la cámara con respecto al UAV
     * @param pos   posición del uav/cámara con respecto al marcador
     * @param eul   orientación del uav con respecto al marcador. El orden de los elementos son 0: roll, 1: pitch, 2: yaw
     * @param rvec  Vector de rotación del marcador con respecto a los ejes de la cámara
     * @param tvec  Vector de translación del marcador con respecto a los ejes de la cámara
     */

    Eigen::Vector3d     pos_marker_in_camera(tvec[0],tvec[1],tvec[2]);
    
    // Transformación de vector de rotación a matriz de rotación
    cv::Mat                     rot_mat;
    Eigen::Matrix3d             rot_mat_marker_from_camera; 
    Rodrigues(rvec,rot_mat);
    cv::cv2eigen(rot_mat, rot_mat_marker_from_camera);

    // La inversa de una matriz de rotación es igual a su traspuesta
    Eigen::Matrix3d     rot_mat_camera_from_marker = rot_mat_marker_from_camera.transpose() ; 

    // Se obtiene la posición del marcador en unos ejes paralelos al marcador centrados en la cámara
    Eigen::Vector3d     pos_marker_in_marker_axis = rot_mat_camera_from_marker*pos_marker_in_camera;

    // Si queremos que la posición esté centrada en el marcador y no en la cámara, es necesario negarla
    pos = -pos_marker_in_marker_axis;
    
    // Aquí debemos de tener en cuenta la rotación de la cámara con respecto al uav. Esta es de 180º alrededor del eje z. 
    // Queremos rotar en ejes absolutos y no en los ejes de rot_mat_marker_from_camera, por lo tanto premultiplicamos. 
    Eigen::Matrix3d             rot_mat_camera_from_uav;
    rot_mat_camera_from_uav     << -1,   0,   0,
                                    0,  -1,   0,
                                    0,   0,   1;
    Eigen::Matrix3d             rot_mat_marker_from_uav = rot_mat_camera_from_uav * rot_mat_marker_from_camera; 

    // Se obtiene la orientación del uav visto desde el marcador
    Eigen::Matrix3d             rot_mat_uav_from_marker = rot_mat_marker_from_uav.transpose() ; 

    // Se obtiene los ángulos de Tait–Bryan en el orden Z-Y-X (ángulos de euler)
    eul = rotationMatrixToEulerAngles(rot_mat_uav_from_marker);

    #ifdef ROT_POS_ORI
    // Transformaciones después de invertir la posición y la orientación. Queremos que la posición y la orientación del UAV
    // esté expresado en un sistema de referencia con su eje z apuntando hacia abajo. El del marcador apunta hacia arriba, así 
    // que se rotará 180º en el eje x
    Eigen::Matrix3d             rot_mat_marker_from_NED;
    rot_mat_marker_from_NED <<  1,   0,   0,
                                0,  -1,   0,
                                0,   0,  -1;    
    Eigen::Matrix3d             rot_mat_uav_from_NED =  rot_mat_marker_from_NED * rot_mat_uav_from_marker; 

    eul = rotationMatrixToEulerAngles(rot_mat_uav_from_NED);
    pos = rot_mat_marker_from_NED * -pos_marker_in_marker_axis;
    #endif

}

bool VisionClass::readVisionParameters(string filename) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;

    fs["camera_parameters"] >> calibration_file;
    fs["video_file"] >> video_file;
    open_window = (string)fs["open_window"]=="true";
    show_rejected = (string)fs["show_rejected"]=="true";
    charuco = (string)fs["charuco"]=="true";
    refindStrategy = (string)fs["refindStrategy"]=="true";
    marker_length = (float)fs["marker_length"];
    dict_type = (int)fs["dict_type"];
    exposure_time = (int)fs["exposure_time"];
    fps = (int)fs["fps"];
    squaresX = (int)fs["squaresX"];
    squaresY = (int)fs["squaresY"];
    marker_length_ch = (float)fs["markerLength"];
    square_length = (float)fs["squareLength"];
    squaresY = (int)fs["squaresY"];
    frame_height = (int)fs["frame_height"];
    frame_width = (int)fs["frame_width"];
    diamond = (string)fs["diamond"]=="true";
    autoScale = (string)fs["autoScale"]=="true";
    write_images = (string)fs["write_images"]=="true";

    // Print them
    cout << "Parámetros de la visión:" <<  endl;
    cout << "\tEl fichero de calibracion es:\t" <<  calibration_file << endl;
    cout << "\tArchivo de video:\t\t" <<  video_file << endl;
    cout << "\tActivación de la ventana:\t" <<  open_window << endl;
    cout << "\tMostrar rechazados:\t\t" <<  show_rejected << endl;
    cout << "\tTamaño del marcador:\t\t" <<  marker_length << endl;
    cout << "\tTipo de diccionario:\t\t" <<  dict_type << endl;
    cout << "\tTiempo de exposición:\t\t" <<  exposure_time << endl;
    cout << "\tFPS:\t\t\t\t" <<  fps << endl;
    if (charuco){
        cout << endl;
        cout << "Charuco:" << endl;
        cout << "\trefindStrategy:\t\t" <<  refindStrategy << endl;
        cout << "\tmarkerLength:\t\t" <<  marker_length_ch << endl;
        cout << "\tsquareLength:\t\t" <<  square_length << endl;
        cout << "\tsquaresX:\t\t" <<  squaresX << endl;
        cout << "\tsquaresY:\t\t" <<  squaresY << endl;
    }
    else if(diamond){
        cout << endl;
        cout << "Diamond:" << endl;
        cout << "\tautoScale:\t\t" <<  autoScale << endl;
    }

    cout << endl;
    return true;
}

bool VisionClass::readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}
