#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
using namespace cv;
using namespace std;

#define WAIT_KEY_MILL      1 // tiempo de espera entre fotogramas cuando se abre la ventana, si vale 0, solo avanza cuando se presiona alguna tecla
//#define WRITE_IMAGES // very slow!

static bool readVisionParameters(string filename, string &calibration_file, string &video_file, bool &open_window, bool &show_rejected, float &marker_length, int &dict_type, int &exposure_time, int &fps  ) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;

    fs["camera_parameters"] >> calibration_file;
    fs["video_file"] >> video_file;
    open_window = (string)fs["open_window"]=="true";
    show_rejected = (string)fs["show_rejected"]=="true";
    marker_length = (float)fs["marker_length"];
    dict_type = (int)fs["dict_type"];
    exposure_time = (int)fs["exposure_time"];
    fps = (int)fs["fps"];

    // Print them
    cout << "Parámetros de la visión:" <<  endl;
    cout << "\tEl fichero de calibracion es:\t" <<  calibration_file << endl;
    cout << "\tArchivo de video:\t\t" <<  video_file << endl;
    cout << "\tActivación de la ventana:\t" <<  open_window << endl;
    cout << "\tMostrar rechazados:\t\t" <<  show_rejected << endl;
    cout << "\tTamaño del marcador:\t\t" <<  marker_length << endl;
    cout << "\tTipo de diccionario:\t\t" <<  dict_type << endl;
    cout << "\tTiempo de exposición:\t\t" <<  exposure_time << endl;
    cout << "\tFPS:\t\t\t" <<  fps << endl;
    cout << endl;
    return true;
}
static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
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

class VisionClass {
    public:
	VisionClass(){
        bool readOk = readVisionParameters("../vision_params.yml", calibration_file, video_file, open_window, show_rejected, marker_length, dict_type, exposure_time, fps);
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
    	//std::cout << "Corner refinement method (0: None, 1: Subpixel, 2:contour, 3: AprilTag 2): " << detectorParams->cornerRefinementMethod << std::endl;

    	dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dict_type));

    	readOk = readCameraParameters(calibration_file, camMatrix, distCoeffs);
    	if(!readOk) {
    	    cerr << "Invalid camera file" << endl;
    	    exit(0);
    	}

        if (video_file!=""){
    		inputVideo.open(video_file);
        }
        else{
    		inputVideo.open(0);
            string cmd;
            if (fps!=0){
                cmd="v4l2-ctl -d /dev/video0 -p "+ fps;
                const char* aux1=cmd.data();
		        system(aux1);
            }
            if (exposure_time!=0){
                cmd="v4l2-ctl -d /dev/video0 -c auto_exposure=1 -c exposure_time_absolute="+ exposure_time;
                const char* aux2=cmd.data();
		        system(aux2);
            }
        }
	}
	void grab_and_retrieve_image(){
       	    inputVideo.grab();
            inputVideo.retrieve(image);
	}
	bool detect_marker(Eigen::Vector3d &pos, Eigen::Vector3d &eul);
    private: 	
        Mat image;
        Mat imageCopy;
    	VideoCapture inputVideo;
        bool isRotationMatrix(Mat &R);
        Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R);
	    static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs); 
        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        vector< Vec3d > rvecs, tvecs;
    	Ptr<aruco::Dictionary> dictionary;
        Ptr<aruco::DetectorParameters> detectorParams;
    	Mat camMatrix; 
	    Mat distCoeffs;
        string calibration_file;
        string video_file;
        bool open_window;
        bool show_rejected;
        float marker_length;
        int dict_type;
        int exposure_time;
        int fps;
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
    aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
    bool found_marker=ids.size() > 0;
    if(found_marker)
        aruco::estimatePoseSingleMarkers(corners, marker_length, camMatrix, distCoeffs, rvecs, tvecs);
        

     if (open_window){
        // draw results
        image.copyTo(imageCopy);
        //imageCopy=image;
        if(found_marker){
           aruco::drawDetectedMarkers(imageCopy, corners, ids);
           aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[0], tvecs[0], marker_length * 0.5f);
        }

        if(show_rejected && rejected.size() > 0)
            aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));
        imshow("out", imageCopy);
        waitKey( WAIT_KEY_MILL );
        //char key = (char)waitKey( WAIT_KEY_MILL );
        //if(key == 27) break;
     }
     #ifdef WRITE_IMAGES
     //char path [30];
     //sprintf(path,"./images/image%d.png", totalIterations);
     //imwrite(path,image);
     #endif

     if (found_marker){
         // tvecs and rvecs are translation and rotation of the marker with respect camera axis
         // camera position coordinates are the same as autopilot
         
         // rvecs are rotation_vectors. Here it is transformed to rotation matrix
         cv::Mat  rot_mat;
         Rodrigues(rvecs[0],rot_mat);

         Eigen::Matrix3d rot_mat_eig; 
         cv::cv2eigen(rot_mat,rot_mat_eig);


         // rotate 180 degrees in x axis in order to get z point down. Also rotate 180º in z axis
         // Rx=[[1, 0, 0], [0, -1, 0], [0, 0, -1]]
         // Rz=[[-1, 0, 0], [0, -1, 0], [0, 0, 1]]
         Eigen::Matrix3d  xz_rotation;
         xz_rotation << -1,  0,  0,
                         0,  1,  0,
                         0,  0, -1;
         Eigen::Matrix3d  x_rotation;
         x_rotation << 1,   0,   0,
                       0,  -1,   0,
                       0,   0,  -1;
         Eigen::Matrix3d  z_rotation;
         z_rotation <<-1,   0,   0,
                       0,  -1,   0,
                       0,   0,   1;
         Eigen::Matrix3d rot_mat_aux=x_rotation*rot_mat_eig.transpose()*z_rotation; 

         // Get position and rotation of camera in marker axis
         Eigen::Vector3d t_in(tvecs[0][0],tvecs[0][1],tvecs[0][2]);
         // TODO: why is neccesary minus sign?
         pos = -rot_mat_aux*t_in;

         eul = rotationMatrixToEulerAngles(rot_mat_aux);
    }
    else{
        pos[0]=pos[1]=pos[2]=NAN;
        eul[0]=eul[1]=eul[2]=NAN;
    }

    return found_marker;
}
