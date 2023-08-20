#include <tuple>

std::tuple<double,double,double,double> ransacCircleLSF(const std::vector<utfr_msgs::msg::Cone> &cones, 
    double radius) {
  int n = cones.size();
  MatrixXd A(n,3);
  //MatrixXd A(n,2);
  MatrixXd b(n,1);

  for (int i = 0; i < n; i++) {
    A(i, 0) = 2.0*cones[i].pos.y;
    A(i, 1) = 2.0*cones[i].pos.x;
    A(i, 2) = -1.0; 
    //b(i, 0) = pow((cones[i].pos.x),2)+pow((cones[i].pos.y),2);
    b(i, 0) = pow((cones[i].pos.x),2)+pow((cones[i].pos.y),2)-pow(radius,2);
  }

  MatrixXd At = A.transpose();
  MatrixXd res = (At * A).inverse() * (At * b);
  double xc = res(0);
  double yc = res(1);
  
  double radiustot;
  for (int i = 0; i<n; i++) {
    radiustot += sqrt(pow((xc-cones[i].pos.x),2)+pow((yc-cones[i].pos.y),2));
  }
  
  double radiusf = radiustot/n;
  
  std::tuple<double,double,double,double> circle;
  circle = std::make_tuple(xc,yc,radius,radiusf);
  return circle;
}

std::vector<double, double> centerBigOrange(){
  continue; // just get the center of the four big orange cones in skidpa
}

std::bool ransacCheck(){ //just the logic to find the small circle on right side and large circle on right side, does not work but can base off of the old code here
  if (left_size > 2 && right_size > 2)
  {
    for (int i = 0; i < left_size; i++)
    {
      for (int j = 1; j < left_size; j++)
      {
        for (int k = 2; k < left_size; k++)
        {
          if (i != j && j != k && i != k)
          {
            utfr_msgs::msg::ConeMap cur_test_left;
            utfr_msgs::msg::Cone cur_test_cone;
            cur_test_cone.type = utfr_msgs::msg::Cone::UNKNOWN;

            int insideThresholdLeft = 0;
            std::tuple<double, double, double, double> circle;

            cur_test_cone.pos.x = 
                curr_cone_detections_->left_cones[i].pos.x;
            cur_test_cone.pos.y = 
                curr_cone_detections_->left_cones[i].pos.y;
            (cur_test_left.left_cones).push_back(cur_test_cone);
            cur_test_cone.pos.x = 
                curr_cone_detections_->left_cones[j].pos.x;
            cur_test_cone.pos.y = 
                curr_cone_detections_->left_cones[j].pos.y;
            (cur_test_left.left_cones).push_back(cur_test_cone);
            cur_test_cone.pos.x = 
                curr_cone_detections_->left_cones[k].pos.x;
            cur_test_cone.pos.y = 
                curr_cone_detections_->left_cones[k].pos.y;
            (cur_test_left.left_cones).push_back(cur_test_cone);
            if (turning == 0){
              circle = util::ransacCircleLSF(cur_test_left.left_cones,small_radius_);
            }
            else if (turning == 1){
              circle = util::ransacCircleLSF(cur_test_left.left_cones,big_radius_);
            }
            r1 = std::get<2>(circle);
            xc1 = std::get<1>(circle);
            yc1 = std::get<0>(circle);
            double outer_threshold_left = r1 + threshold_radius_;
            double inner_threshold_left = r1 - threshold_radius_;
            for (int a = 0; a < left_size; a++)
            {
              if (inner_threshold_left < sqrt(pow((std::get<1>(circle) -
                  curr_cone_detections_->left_cones[a].pos.x), 2) +
                  pow((std::get<0>(circle) - curr_cone_detections_->left_cones[a].pos.y), 2)) &&
                  outer_threshold_left > sqrt(pow((std::get<1>(circle) -
                  curr_cone_detections_->left_cones[a].pos.x),2) +
                  pow((std::get<0>(circle) - curr_cone_detections_->left_cones[a].pos.y),2)))
              {
                insideThresholdLeft += 1;
              }
            }
            if (((turning == 0 && insideThresholdLeft >= threshold_cones_ && yc1<0)
                || (insideThresholdLeft >= threshold_cones_ && yc1>0 && turning == 1)) 
                && xc1<3.0){
              leftFind = true;
            }
          }
          if (leftFind == true)
          {
            break;
          }
        }
        if (leftFind == true)
        {
          break;
        }
      }
      if (leftFind == true)
      {
        break;
      }
    } 
  }
}

std::vector<double, double> get_center_left(std::vector<double> center_right, std::vector<double> center_center){
  double dx = center_center[0] - center_right[0];
  double dy = center_center[1] - center_right[1];
  std::vector<double> center_left;
  center_left[0] = center_center[0] + dx;
  center_left[1] = center_center[1] + dy;
  return center_left;
}

double get_center_orientation(std::vector<double> center_right, std::vector<double> center_left){
  double dx = center_right[0] - center_left[0];
  double dy = center_right[1] - center_left[1];
  double theta = atan2(dy,dx);
  double orientation = M_PI - theta;
  return orientation;
}

std::vector<double, double, double> getSkidpadCenter(std::vector<std::vector<double, double, double>> global_cone_map, int num_left_cones, int num_right_cones){
  //(x,y) center of the right circle
  std::vector<double, double> center_right = ransacCheck(); //would return the center of the right circles
  std::vector<double, double> center_center = centerBigOrange();
  std::vector<double, double> center_left = get_center_left(center_right, center_center);
  double orientation = get_center_orientation(center_right, center_left);
  std::vector<double, double, double> skidpad_center;
  skidpad_center[0] = center_center[0];
  skidpad_center[1] = center_center[1];
  skidpad_center[2] = orientation;
  return skidpad_center;
}

std::vector<double, double, double> origin_frame_to_skidpad_frame_transform(std::vector<double> point){
  //point is a vector of x,y,theta
  //function converts point that is in origin frame of (0,0,0) to skidpad frame
  std::vector<double, double, double> skidpad_center = getSkidpadCenter();
  double x = point[0] - skidpad_center[0];
  double y = point[1] - skidpad_center[1];
  double theta = point[2] - skidpad_center[2];
  std::vector<double, double, double> skidpad_point;
  skidpad_point[0] = x;
  skidpad_point[1] = y;
  skidpad_point[2] = theta;
  return skidpad_point;
}

std::vector<std::vector<double>> origin_frame_spline_to_skidpad_frame(std::vector<std::vector<double>> spline_params){
  //function takes spline params in the form
  //spline_params[0] is x(t) coefficients and would be x(t) = a * x ** 5 + b * x ** 4 + c * x ** 3 + d * x ** 2 + e * x + f
  //spline_params[1] is y(t) coefficients and would be y(t) = a * y ** 5 + b * y ** 4 + c * y ** 3 + d * y ** 2 + e * y + f
  //function then converts the spline params to skidpad frame
  std::vector<std::vector<double>> skidpad_spline_params;
  std::vector<double> x_params = spline_params[0];
  std::vector<double> y_params = spline_params[1];
  std::vector<double> skidpad_x_params;
  std::vector<double> skidpad_y_params;
  std::vector<double, double, double> skidpad_center = getSkidpadCenter();
  for (int i = 0; i < x_params.size(); i++){
    skidpad_x_params[i] = x_params[i] - skidpad_center[0];
    skidpad_y_params[i] = y_params[i] - skidpad_center[1];
  }
  skidpad_spline_params[0] = skidpad_x_params;
  skidpad_spline_params[1] = skidpad_y_params;
  return skidpad_spline_params;
}
