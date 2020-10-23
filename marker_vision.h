#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
using namespace cv;
using namespace std;

#define DICTIONARY 10   // 6x6 256
//#define MARKER_LENGTH 0.173 
#define MARKER_LENGTH 0.179 
// Corner refinement: CORNER_REFINE_NONE=0, CORNER_REFINE_SUBPIX=1," "CORNER_REFINE_CONTOUR=2, CORNER_REFINE_APRILTAG=3}"
#define CALIBRATION_PARAMETERS "calibration_parameters.txt"
#define REFINEMENT_METHOD 1
#define SHOW_REJECTED  false
#define DRAW_AXIS 
#define OPEN_WINDOW


class VisionClass {
    public:
	VisionClass(){
    		/*** Vision setup ***/
    		detectorParams = aruco::DetectorParameters::create();

    		//override cornerRefinementMethod read from config file
    		detectorParams->cornerRefinementMethod = REFINEMENT_METHOD;
    		std::cout << "Corner refinement method (0: None, 1: Subpixel, 2:contour, 3: AprilTag 2): " << detectorParams->cornerRefinementMethod << std::endl;

    		dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(DICTIONARY));

    		    bool readOk = readCameraParameters(CALIBRATION_PARAMETERS, camMatrix, distCoeffs);
    		    if(!readOk) {
    		        cerr << "Invalid camera file" << endl;
    		        exit(0);
    		    }

    		inputVideo.open(0);
    		inputVideo.set(cv::CAP_PROP_MONOCHROME, 1);
	}
	void grab_and_retrieve_image(){
       	    inputVideo.grab();
            inputVideo.retrieve(image);
	}
	bool detect_marker(Eigen::Vector3d pos, Eigen::Vector3d eul);
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

// Checks if a matrix is a valid rotation matrix.
bool VisionClass::isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());
    
    return  norm(I, shouldBeIdentity) < 1e-6;
    
}

bool VisionClass::readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}


bool VisionClass::detect_marker(Eigen::Vector3d pos, Eigen::Vector3d eul){
    aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
    bool found_marker=ids.size() > 0;
    if(found_marker)
        aruco::estimatePoseSingleMarkers(corners, MARKER_LENGTH, camMatrix, distCoeffs, rvecs, tvecs);
        

     #ifdef DRAW_AXIS
        // draw results
        image.copyTo(imageCopy);
        //imageCopy=image;
        if(found_marker){
           aruco::drawDetectedMarkers(imageCopy, corners, ids);
           aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[0], tvecs[0], MARKER_LENGTH * 0.5f);
        }

        if(SHOW_REJECTED && rejected.size() > 0)
            aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));
     #endif
     #ifdef OPEN_WINDOW
        imshow("out", imageCopy);
     #endif

     if (found_marker){
         // tvecs and rvecs are translation and rotation of the marker with respect camera axis
         // camera position coordinates are the same as autopilot
         
         // rvecs are rotation_vectors. Here it is transformed to rotation matrix
         cv::Mat  rot_mat;
         Rodrigues(rvecs[0],rot_mat);
         //if (!isRotationMatrix(rot_mat)){
         //    break;
         //}
         //Vec3d euler_angles = rotationMatrixToEulerAngles(rot_mat);

         //Eigen::Matrix3d rot_mat_eig, rot_mat_aux2; 
         Eigen::Matrix3d rot_mat_eig; 
         cv::cv2eigen(rot_mat,rot_mat_eig);
         //rot_mat_eig=-rot_mat_aux2; 


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

    return found_marker;
}
