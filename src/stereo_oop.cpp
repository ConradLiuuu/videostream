#include <ros/ros.h>
#include <std_msgs/Int64MultiArray.h>
#include <iostream>
#include <math.h>
#include <thread>
using namespace std;
using namespace ros;

string camera_L = "left", camera_R = "right";
// left camera intrinsic parameter
double fu_L = 1768.02372, fv_L = 1763.31154;  // focal length
double u0_L = 1052.58192, v0_L = 805.05924;  // principal point
double kc_L[8] = {0.00750142983253570, -0.0182942738235753, -0.00124094441332947, -0.00226988800017158, -0.00562121138720920, 0.0148026465858007, 0.000718693812894019, 0.00166704024044161};
// right camera intrinsic parameter
double fu_R = 1777.62991, fv_R = 1772.32616; // focal length
double u0_R = 1090.97384, v0_R = 803.15273; // principal point
double kc_R[8] = {-0.00253711008550057, -0.0249585312796035, -0.00212056899607580, -0.00549252029661931, 0.00294598049096739, 0.0181609205664959, 0.00119360905595092, 0.00432458235751296};

//double a = 0,b = 0, c = 0, dd = 0;
int dis_Ix_L = -1, dis_Iy_L = -1, dis_Ix_R = -1, dis_Iy_R = -1;
double Ix_L, Iy_L, Ix_R, Iy_R;
//int ID_L, ID_R;
int ID_L = 0, ID_R = 0;
int id_L, id_R;
bool isDone = true;
int bound = -26;

class Sub_ball_center
{
private:
  NodeHandle nh;
  Subscriber sub_left, sub_right;

public:
  Sub_ball_center(){
    sub_left = nh.subscribe("ball_center_left", 1, &Sub_ball_center::callback_left, this);
    sub_right = nh.subscribe("ball_center_right", 1, &Sub_ball_center::callback_right, this);
  }

  void operator()(){
    sub_left = nh.subscribe("ball_center_left", 1, &Sub_ball_center::callback_left, this);
    sub_right = nh.subscribe("ball_center_right", 1, &Sub_ball_center::callback_right, this);
    ros::spin();
  }

  void callback_left(const std_msgs::Int64MultiArray::ConstPtr& msg_left)
  {
    if ((msg_left->data[1] >= 0) && (msg_left->data[2] >= 0)){
      ID_L = msg_left->data[0];
      dis_Ix_L = msg_left->data[1];
      dis_Iy_L = msg_left->data[2];
      //cout << "ID_L = " << ID_L << endl;
      //isDone = false;
    }
    else {
      ID_L = 0;
      dis_Ix_L = -1;
      dis_Iy_L = -1;
    }

    //cout << "dis_Ix_L = " << dis_Ix_L << endl;
    //cout << "dis_Iy_L = " << dis_Iy_L << endl;
  }

  void callback_right(const std_msgs::Int64MultiArray::ConstPtr& msg_right)
  {
    if ((msg_right->data[1] >= 0) && (msg_right->data[2] >= 0)){
      ID_R = msg_right->data[0];
      dis_Ix_R = msg_right->data[1];
      dis_Iy_R = msg_right->data[2];
      //cout << "ID_R = " << ID_R << endl;
      //isDone = false;
    }
    else{
      ID_R = -1;
      dis_Ix_R = -1;
      dis_Iy_R = -1;
    }

    //cout << "dis_Ix_R = " << dis_Ix_R << endl;
    //cout << "dis_Iy_R = " << dis_Iy_R << endl;
  }
};

void correction_img(string camera, int dis_Ix, int dis_Iy, double fu, double fv, double u0, double v0, double kc[8])
{
  double dis_hxz, dis_hyz, rd ,G, hxz, hyz;

  if (camera == "left"){
    // calculate distortion ray vector
    dis_hxz = (dis_Ix - u0) / fu;
    dis_hyz = (dis_Iy - v0) / fv;
    //cout << "distortion hxz_L = " << dis_hxz << endl;
    //cout << "distortion hyz_L = " << dis_hyz << endl;


    // calcuate correction parameter
    rd = sqrt(pow(dis_hxz,2)+pow(dis_hyz,2));
    G = 4*kc[4]*pow(rd,2) + 6*kc[5]*pow(rd,4) + 8*kc[6]*dis_hyz + 8*kc[7]*dis_hxz + 1;

    // calculate correction sight vector
    hxz = dis_hxz + (1/G)*(kc[0]*pow(rd,2)+kc[1]*pow(rd,4)*dis_hxz+2*kc[2]*dis_hxz*dis_hyz+kc[3]*(pow(rd,2)+2*pow(dis_hxz,2)));
    hyz = dis_hyz + (1/G)*(kc[0]*pow(rd,2)+kc[1]*pow(rd,4)*dis_hyz+kc[2]*(pow(rd,2)+2*pow(dis_hyz,2))+2*kc[3]*dis_hxz*dis_hyz);

    // calculate correction position
    Ix_L = u0 + fu*hxz;
    Iy_L = v0 + fv*hyz;
    //cout << "correct Ix_L Iy_L = [" << Ix_L << "," << Iy_L << "]" << endl;
    //cout << "Ix_L by func = " << Ix_L << endl;
    //cout << "Iy_L by func= " << Iy_L << endl;

  }

  if (camera == "right"){
    // calculate distortion sight vector
    dis_hxz = (dis_Ix - u0) / fu;
    dis_hyz = (dis_Iy - v0) / fv;

    // calcuate correction parameter
    rd = sqrt(pow(dis_hxz,2)+pow(dis_hyz,2));
    G = 4*kc[4]*pow(rd,2) + 6*kc[5]*pow(rd,4) + 8*kc[6]*dis_hyz + 8*kc[7]*dis_hxz + 1;

    // calculate correction ray vector
    hxz = dis_hxz + (1/G)*(kc[0]*pow(rd,2)+kc[1]*pow(rd,4)*dis_hxz+2*kc[2]*dis_hxz*dis_hyz+kc[3]*(pow(rd,2)+2*pow(dis_hxz,2)));
    hyz = dis_hyz + (1/G)*(kc[0]*pow(rd,2)+kc[1]*pow(rd,4)*dis_hyz+kc[2]*(pow(rd,2)+2*pow(dis_hyz,2))+2*kc[3]*dis_hxz*dis_hyz);

    // calculate correction position
    Ix_R = u0 + fu*hxz;
    Iy_R = v0 + fv*hyz;
    //cout << "correct Ix_R Iy_R = [" << Ix_R << "," << Iy_R << "]" << endl;
    //cout << "Ix_R by func = " << Ix_R << endl;
    //cout << "Iy_R by func= " << Iy_R << endl;

  }

}

void tt(){

  double R_R2L[3][3] = {{0.998342653395923, -0.0563204736316192, -0.0118300743995107},{0.0562365219029644, 0.998390689004778, -0.00731339270257675},{0.0122229298715684, 0.00663598963793509, 0.999903277135784}}; // matlab given
  double R_L2R[3][3] = {{R_R2L[0][0], R_R2L[1][0], R_R2L[2][0]},{R_R2L[0][1], R_R2L[1][1], R_R2L[2][1]},{R_R2L[0][2], R_R2L[1][2], R_R2L[2][2]}};

  double b_R2L[3] = {-310.52772, -18.47588, 7.00509}; // matlab given
  double b_L2R[3] = {310.966464444269, 0.910592592369968, -10.8130998443072}; // -R_L2R * b_R2L

  double d[3];
  d[0] = (R_R2L[0][0]*b_L2R[0]) + (R_R2L[0][1]*b_L2R[1]) + (R_R2L[0][2]*b_L2R[2]);
  d[1] = (R_R2L[1][0]*b_L2R[0]) + (R_R2L[1][1]*b_L2R[1]) + (R_R2L[1][2]*b_L2R[2]);
  d[2] = (R_R2L[2][0]*b_L2R[0]) + (R_R2L[2][1]*b_L2R[1]) + (R_R2L[2][2]*b_L2R[2]);

  double b_L2W[3] = {-442.694030024999, 645.423910795827, 1976.74826550359};
  double R_L2W[3][3] = {{0.999849820654844, -0.0171622592318973, -0.00240686404545828}, {-0.0124560895423012, -0.615117028697886, -0.788337419408211}, {0.0120491480939666, 0.788249007524869, -0.615238425463063}};
  double R_W2L[3][3] = {{R_L2W[0][0], R_L2W[1][0], R_L2W[2][0]},{R_L2W[0][1], R_L2W[1][1], R_L2W[2][1]},{R_L2W[0][2], R_L2W[1][2], R_L2W[2][2]}};

  double hx_L, hy_L, hz_L;

  double hx_W, hy_W, hz_W;
  double dif_L2W[3] = {0};

  double y1;
  double y2 = 0;
  int diff = 0;

  while (ros::ok()) {
    //ROS_INFO("dis_Ix_L = %f, dis_Iy_L = %f \n", a,b);
    //ROS_INFO("dis_Ix_R = %f, dis_Iy_R = %f \n", c,dd);
    //cout << dis_Ix_L << "," << dis_Iy_L << "|" << dis_Ix_R << "," << dis_Iy_R << endl;
    //cout << "ID_L = " << ID_L  << ", ID_R = " << ID_R << endl;

    //diff = ID_L - ID_R;
    id_L = ID_L;
    id_R = ID_R;

    if (id_L == id_R){
      if ((dis_Ix_L >= 0) && (dis_Iy_L >= 0) && (dis_Ix_R >= 0) && (dis_Iy_R >= 0) && (ID_L == ID_R)){
        correction_img(camera_L, dis_Ix_L, dis_Iy_L, fu_L, fv_L, u0_L, v0_L, kc_L);
        correction_img(camera_R, dis_Ix_R, dis_Iy_R, fu_R, fv_R, u0_R, v0_R, kc_R);

        // calcuate k
        double k = ((R_R2L[0][0]*(Ix_L-u0_L)/fu_L) + (R_R2L[0][1]*(Iy_L-v0_L)/fv_L) + R_R2L[0][2]) - ((Ix_R-u0_R)/fu_R)*((R_R2L[2][0]*(Ix_L-u0_L)/fu_L) + (R_R2L[2][1]*(Iy_L-v0_L)/fv_L) + R_R2L[2][2]);

        // calculate left ray vector
        hz_L = (d[0] - (d[2]*(Ix_R-u0_R)/fu_R)) / k;
        hx_L = hz_L*(Ix_L-u0_L)/fu_L;
        hy_L = hz_L*(Iy_L-v0_L)/fv_L;

        dif_L2W[0] = hx_L-b_L2W[0];
        dif_L2W[1] = hy_L-b_L2W[1];
        dif_L2W[2] = hz_L-b_L2W[2];

        hx_W = R_W2L[0][0]*dif_L2W[0] + R_W2L[0][1]*dif_L2W[1] + R_W2L[0][2]*dif_L2W[2];
        hy_W = R_W2L[1][0]*dif_L2W[0] + R_W2L[1][1]*dif_L2W[1] + R_W2L[1][2]*dif_L2W[2];
        hz_W = R_W2L[2][0]*dif_L2W[0] + R_W2L[2][1]*dif_L2W[1] + R_W2L[2][2]*dif_L2W[2];
        //cout << "object position = [" << hx_W / 10 << "," << hy_W / 10 << "," << hz_W / 10 << "]" << endl;

        hx_W = R_W2L[0][0] * dif_L2W[0] + R_W2L[0][1] * dif_L2W[1] + R_W2L[0][2] * dif_L2W[2] - 17.6031;
        hy_W = R_W2L[1][0] * dif_L2W[0] + R_W2L[1][1] * dif_L2W[1] + R_W2L[1][2] * dif_L2W[2] - (-23.1113);
        hz_W = R_W2L[2][0] * dif_L2W[0] + R_W2L[2][1] * dif_L2W[1] + R_W2L[2][2] * dif_L2W[2] - 18.5448 + 16;

        y1 = hy_W;
        //diff = y1-y2;
        if ((y1 != y2) && (ID_L == ID_R )){
          //ROS_INFO("ID_L = %d, ID_R = %d", ID_L, ID_R);
          cout << "ID_L = " << ID_L  << ", ID_R = " << ID_R << endl;;
          ROS_INFO("correction position = [%f, %f, %f]", hx_W / 10, hy_W / 10,hz_W / 10);
          //cout << "correction position = [" << hx_W / 10 << "," << hy_W / 10 << "," << hz_W / 10 << "]" << endl;
          y2 = y1;
        }
        isDone = true;
      }

    }

/*
    if ((dis_Ix_L >= 0) && (dis_Iy_L >= 0) && (dis_Ix_R >= 0) && (dis_Iy_R >= 0)){
      correction_img(camera_L, dis_Ix_L, dis_Iy_L, fu_L, fv_L, u0_L, v0_L, kc_L);
      correction_img(camera_R, dis_Ix_R, dis_Iy_R, fu_R, fv_R, u0_R, v0_R, kc_R);

      // calcuate k
      double k = ((R_R2L[0][0]*(Ix_L-u0_L)/fu_L) + (R_R2L[0][1]*(Iy_L-v0_L)/fv_L) + R_R2L[0][2]) - ((Ix_R-u0_R)/fu_R)*((R_R2L[2][0]*(Ix_L-u0_L)/fu_L) + (R_R2L[2][1]*(Iy_L-v0_L)/fv_L) + R_R2L[2][2]);

      // calculate left ray vector
      hz_L = (d[0] - (d[2]*(Ix_R-u0_R)/fu_R)) / k;
      hx_L = hz_L*(Ix_L-u0_L)/fu_L;
      hy_L = hz_L*(Iy_L-v0_L)/fv_L;

      dif_L2W[0] = hx_L-b_L2W[0];
      dif_L2W[1] = hy_L-b_L2W[1];
      dif_L2W[2] = hz_L-b_L2W[2];

      hx_W = R_W2L[0][0]*dif_L2W[0] + R_W2L[0][1]*dif_L2W[1] + R_W2L[0][2]*dif_L2W[2];
      hy_W = R_W2L[1][0]*dif_L2W[0] + R_W2L[1][1]*dif_L2W[1] + R_W2L[1][2]*dif_L2W[2];
      hz_W = R_W2L[2][0]*dif_L2W[0] + R_W2L[2][1]*dif_L2W[1] + R_W2L[2][2]*dif_L2W[2];
      //cout << "object position = [" << hx_W / 10 << "," << hy_W / 10 << "," << hz_W / 10 << "]" << endl;

      hx_W = R_W2L[0][0] * dif_L2W[0] + R_W2L[0][1] * dif_L2W[1] + R_W2L[0][2] * dif_L2W[2] - 17.6031;
      hy_W = R_W2L[1][0] * dif_L2W[0] + R_W2L[1][1] * dif_L2W[1] + R_W2L[1][2] * dif_L2W[2] - (-23.1113);
      hz_W = R_W2L[2][0] * dif_L2W[0] + R_W2L[2][1] * dif_L2W[1] + R_W2L[2][2] * dif_L2W[2] - 18.5448 + 16;

      y1 = hy_W;
      //diff = y1-y2;
      if (y1 != y2){
        cout << "ID_L = " << ID_L  << ", ID_R = " << ID_R << endl;;
        cout << "correction position = [" << hx_W / 10 << "," << hy_W / 10 << "," << hz_W / 10 << "]" << endl;
        y2 = y1;
      }

    }
    */
    /*else{
      cout << "not found ball" << endl;
    }*/

  }

}

int main(int argc, char** argv)
{
  init(argc, argv, "stereo_oop");
  //Stereo sterro;
  Sub_ball_center sub;

  thread t1(ref(sub));
  thread t2(tt);

  t2.join();
  t1.join();


  return 0;
}
