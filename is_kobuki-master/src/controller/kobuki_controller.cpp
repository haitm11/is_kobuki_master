#include "../../include/controller/kobuki_controller.hpp"
#include "../ccd/ccdstar.cpp"

cell **cells;

// Chuyen mapdata tu 1 chieu sang 2 chieu
int** KobukiController::oneArrToTwoArr(std::vector<int> mapData, int width, int height) {
  int **mapArr = (int **)malloc(height * sizeof(int *));
  for (int i = 0; i < height; i++)
    mapArr[i] = (int *)malloc(width * sizeof(int));

  int row = 0;
  int col = width-1;

  for (int i = mapData.size()-1; i >= 0; i--) {
    mapArr[row][col] = mapData[i];
    col--;
    if (col == -1) {
      row++;
      col = width-1;
    }
  }

  return mapArr;
}

/* Sau khi quet ban do, ta duoc 1 file ban do. Trong file ban do nay, se co nhung phan khong co y nghia
Ta can loai bo chung. up, left, down, right la gioi han bien cho phan ban do co y nghia
*/
int KobukiController::findLineDown(int **mapArr, int width, int height) {
  int row = 0;
  int col = 0;
  for (row = height - 1; row >= 0; row--) {
    for (col = 0; col < width; col++) {
      if (mapArr[row][col] == 100)
        return row;
    }
  }
  return 0;
}

int KobukiController::findLineUp(int **mapArr, int width, int height) {
  int row = 0, col = 0;
  for (row = 0; row < height; row++) {
    for (col = 0; col < width; col++) {
      if (mapArr[row][col] == 100)
        return row;
    }
  }
  return 0;
}

int KobukiController::findLineLeft(int **mapArr, int width, int height) {
  int row = 0, col = 0;
  for (col = 0; col < width; col++) {
    for (row = 0; row < height; row++) {
      if (mapArr[row][col] == 100)
        return col;
    }
  }
  return 0;
}

int KobukiController::findLineRight(int **mapArr, int width, int height) {
  int row = 0, col = 0;
  for (col = width - 1; col >= 0; col--) {
    for (row = 0; row < height; row++) {
      if (mapArr[row][col] == 100)
        return col;
    }
  }
  return 0;
}

// Dang ky subcriber, publisher
bool KobukiController::init() {
  ROS_INFO("Init Controller");

  /* Ta co the dieu khien robot di thang, re
  Tuy nhien de co the dieu khien robot di thang bao nhieu m, re goc bao nhieu do
  Ta can subscribe topic /odom de co the lay duoc toa do cua robot. */
  odometry_sub = nh.subscribe("/odom", 1, &KobukiController::odometryHandle, this);

  /* Subscribe topic "scan" de lay thong tin quet cua laser nham phat hien vat can */
  laser_sub = nh.subscribe("/scan", 1, &KobukiController::laserHandle, this);

  /* Viec subscribe topic "amcl_pose" de tinh toan duoc vi tri hien tai cua robot */
  amcl_sub = nh.subscribe("/amcl_pose", 1, &KobukiController::amclHandle, this);

  /* Subscribe topic "map_metadata" de lay thong tin ban do (file .yaml va file anh) */
  map_metadata_sub = nh.subscribe("/map_metadata", 10, &KobukiController::initializeMap, this);

  /* Publisher 
  cmd_vel_pub dung de publish thong tin dieu khien robot di thang, re
  power_pub hien tai chua thay su dung, cung khong nho dung de lam gi, cu de day da
  amcl_pub publish thong tin de hien thi len RVIZ */
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  reset_odom_pub = nh.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry", 1);
  amcl_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

  ROS_INFO("Init Controller Success!");

  // lenh spin goi lai cac loi goi callback
  // Tom lai, co spin thi cac ham subcribe moi duoc goi
  ros::spin();

  return true;
}

float KobukiController::computeHypotenuse(float a, float b) {return std::sqrt(a*a+b*b);}

float KobukiController::getDistanceFromCellToCell(int rowBegin, int colBegin, 
                                                  int rowEnd, int colEnd, float cellSize) {
  float rowDistance = std::abs(rowBegin - rowEnd) * cellSize;
  float colDistance = std::abs(colBegin - colEnd) * cellSize;
  return computeHypotenuse(rowDistance, colDistance);
}

/* Trong he toa do Oxy ma odom sinh ra. Huong cua robot luon co mot tri so nao do.
Khi robot dang o vi tri A, ta muon no den B thi dau tien, ta can quay robot 1 goc de ve phia B,
luc nay, huong cua robot = anpha 
sau do dieu khien robot di thang quang duong AB. Ham nay phuc vu viec tinh goc anpha. 
rowBegin, colBegin la toa do cua A, rowEnd, colEnd la toa do B */
float KobukiController::getAngleFromCellToCell(int rowBegin, int colBegin, int rowEnd, int colEnd) {
  if(rowBegin == rowEnd && colBegin < colEnd) return 0;
  if(rowBegin == rowEnd && colBegin > colEnd) return PI;
  if(rowBegin > rowEnd && colBegin == colEnd) return PI/2;
  if(rowBegin < rowEnd && colBegin == colEnd) return -PI/2;
  if(rowBegin == rowEnd && colBegin == colEnd) return -4;
  float rowDistance = std::abs(rowBegin - rowEnd);
  float colDistance = std::abs(colBegin - colEnd);
  float anpha = std::atan(rowDistance / colDistance);

  if(rowBegin < rowEnd && colBegin < colEnd) return -anpha;
  if(rowBegin > rowEnd && colBegin < colEnd) return anpha;
  if(rowBegin > rowEnd && colBegin > colEnd) return PI-anpha;
  if(rowBegin < rowEnd && colBegin > colEnd) return anpha-PI;
}

/* Trong he toa do Oxy ma odom sinh ra. Huong cua robot luon co mot tri so nao do.
Khi robot dang o vi tri A, ta muon no den B thi dau tien, ta can quay robot 1 goc anpha de ve phia B, 
sau do dieu khien robot di thang quang duong AB. Ham nay phuc vu viec quay den goc anpha.*/
void KobukiController::turnToCell(int r, int c) {
  printf("turn to cell %d %d\n", r,c);
  ros::Rate rate(500);
  geometry_msgs::Twist move;
  move.linear.x = 0;
  float vFirst, angleMiddle, beta;

  float angle = getAngleFromCellToCell(currentCellY - up,currentCellX - left, r, c);
  if(angle == -4) return;

  // xac dinh can quay trai hay quay phai
  float diff = current_pose.theta - angle;
  if(diff >= PI || (diff < 0 && diff > -PI)) move.angular.z = angularSpeed;
  else move.angular.z = -angularSpeed;
  
  if(angle == (float)PI) {
    if(current_pose.theta >=0) {
      move.angular.z = angularSpeed;
    }
    else {
      move.angular.z = -angularSpeed;
      angle = -angle;
    }
  }

  // gia su goc can phai quay mang gia tri anpha thi angleMiddle = anpha / 2
  vFirst = move.angular.z;

  if((current_pose.theta >= 0 && angle >= 0) || (current_pose.theta <= 0 && angle <= 0)) {
    angleMiddle = std::fabs(current_pose.theta - angle) / 2;
  } else {
    angleMiddle = std::fabs(current_pose.theta) + std::fabs(angle);
    if(angleMiddle <= PI) angleMiddle = angleMiddle / 2;
    else angleMiddle = (2*PI-angleMiddle) / 2;
  }
  
  // bat dau quay
  while(std::fabs(current_pose.theta - angle) >= 0.05) {
    cmd_vel_pub.publish(move);
    angle = getAngleFromCellToCell(currentCellY - up,currentCellX - left, r, c);
    if(angle == -4) break;
    // if((current_pose.theta >= 0 && angle >= 0) || (current_pose.theta <= 0 && angle <= 0)) {
    //   beta = std::fabs(current_pose.theta - angle);
    // } else {
    //   beta = std::fabs(current_pose.theta) + std::fabs(angle);
    //   if(beta > PI) beta = (2*PI-beta);
    // }
    // if(beta <= angleMiddle) { 
    //   move.angular.z = vFirst * beta / angleMiddle;
    //   if(std::fabs(move.angular.z) <= (PI / 64)) {
    //     if(move.angular.z >= 0) move.angular.z = PI / 64;
    //     else move.angular.z = -PI / 64;
    //   }
    // }
    // if(beta <= PI / 32) {
    //   if(move.angular.z >= 0) move.angular.z = PI / 64;
    //   else move.angular.z = -PI / 64;
    // } 
    ros::spinOnce();
    rate.sleep();
  }
  
  // stop
  move.angular.z = 0;
  cmd_vel_pub.publish(move);
  printf("end turnToCell\n");
  usleep(2000000);
}

/* Ham nay dieu khien robot quay mot goc degree.
Chi quay duoc cac goc <=180 do, degree la goc muon quay, degree > 0 thi quay trai. */
void KobukiController::turn(float degree) {
  printf("turn %f\n", degree);
  ros::Rate rate(50);
  float theta = current_pose.theta;
  float current_theta = current_pose.theta;
  geometry_msgs::Twist move;
  move.linear.x = 0;
  move.angular.z = degree > 0 ? angularSpeed/2 : -angularSpeed/2;
  std_msgs::Empty emptyMsg;

  float angle = 0;
  int flag = 0;

  while (ros::ok() && (angle < std::abs(degree))) {
    cmd_vel_pub.publish(move);
    ros::spinOnce();
    rate.sleep();
    current_theta = current_pose.theta;

    if((current_theta*theta) < 0 && flag == 0) {
      if(std::abs(current_theta) > (PI-0.5)) flag = -1;
      else flag = 1;
    }

    if(flag == -1) {   
      angle = 2*PI - std::abs(theta) - std::abs(current_theta);
    } else {
      angle = std::abs(current_theta - theta);
    }

  }

  // stop
  move.angular.z = 0;
  cmd_vel_pub.publish(move);
  printf("end turn\n");
  usleep(2000000);
}

void KobukiController::go(float distance) {

  goAnble = true;
  printf("go %f\n", distance);
  ros::Rate rate(50);
  float x = current_pose.x;
  float y = current_pose.y;
  geometry_msgs::Twist move;
  move.angular.z = 0;
  move.linear.x = linearSpeed;
  // move.angular.z = angularSpeed/2;


  while (ros::ok() && computeHypotenuse(current_pose.x - x, current_pose.y - y) < distance) {
    cmd_vel_pub.publish(move);
    ros::spinOnce();
    // sleep 1/50 second
    rate.sleep();
    if(!goAnble) break;
  }
  move.linear.x = 0;
  cmd_vel_pub.publish(move);
  goAnble = false;
  printf("end go\n");
  usleep(2000000);
}

/* Khi bat dau chay chuong trinh. Robot tu sinh ra 1 he toa do Oxy. Goc O la vi tri dau tien cua robot
truc Ox la huong cua robot */
void KobukiController::odometryHandle(const nav_msgs::OdometryConstPtr& odometry) {
  // linear position
  current_pose.x = odometry->pose.pose.position.x;
  current_pose.y = odometry->pose.pose.position.y;

  // quaternion to RPY conversion
  tf::Quaternion q(
    odometry->pose.pose.orientation.x,
    odometry->pose.pose.orientation.y,
    odometry->pose.pose.orientation.z,
    odometry->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // angular position
  current_pose.theta = yaw;

  // printf("odometry %f | %f | %f\n", current_pose.x, current_pose.y, current_pose.theta);
}

/* kiem tra cells[row][col] co gan vat can da xac dinh tu truoc hay khong */
bool KobukiController::checkNearObsReal(int row, int col) {
  if(row < 0 || row >= rows || col < 0 || col >= cols) return true;
  if(cells[row][col].isObReal == 1 || cells[row][col].isObReal == 2) return true;
  
  if((cells[row-1][col].isObReal == 1)
  || (cells[row][col-1].isObReal == 1)
  || (cells[row+1][col].isObReal == 1)
  || (cells[row][col+1].isObReal == 1)
  || (cells[row-1][col-1].isObReal == 1)
  || (cells[row+1][col+1].isObReal == 1)
  || (cells[row-1][col+1].isObReal == 1)
  || (cells[row+1][col-1].isObReal == 1)) return true;

  return false;
}

/* Ham nay chay 1 luong rieng biet voi chuong trinh chinh, chay lien tuc, goi dau len nhau.
Vi vay de tranh xung dot, ta can bien isObs de lam khoa cua. Khi robot phat hien OBS,
no se chi xu ly luong hien tai, cac luong chay sau se bi bo qua */
void KobukiController::laserHandle(const sensor_msgs::LaserScanConstPtr laser) {
  if(!goAnble) return;
  float angleBetween2Ray = (laser->angle_max - laser->angle_min) / laser->ranges.size();
  float anpha;
  float amclPoseXObs, amclPoseYObs;
  int cellXObs, cellYObs;
  int obs_i = -1;
  int left_i, right_i;
  float distance1, distance2, distance;
  int i;
  int kk=0;
  if(!isObs) {
    for(i = (laser->ranges.size()/4); i < (laser->ranges.size()/4*3); i++) {
      if((laser->ranges[i] <= obsDetecDistance)) {
        kk++;
        if(kk>=10) {
          
        printf("laserHandle: OBS\n");
        obs_i = i;
        isObs = true;
        flag = 0;         // bien flag dung de stop chuong trinh chinh, doi laser xu ly xong moi chay tiep
        goAnble = false;
        usleep(500000);
        turn(-PI_MOVE / 2);
        turn(PI_MOVE / 2); 
        break;     
        }       
      }
    }
    if(isObs) {
      printf("laserHandle: finding left_i, right_i and Determine OBS\n");

      if(obs_i < (laser->ranges.size() / 2)) {
        anpha = current_pose.theta - angleBetween2Ray*(laser->ranges.size()/2-obs_i-0.5);
      } else {
        anpha = current_pose.theta + angleBetween2Ray*(obs_i-laser->ranges.size()/2+0.5);
      } 
      amclPoseXObs = amclPoseX + (laser->ranges[obs_i])*(std::cos(anpha));
      amclPoseYObs = amclPoseY + (laser->ranges[obs_i])*(std::sin(anpha));
      cellXObs = (amclPoseXObs - mapOriginX) / mapResolution;
      cellYObs = mapHeight-1-(amclPoseYObs - mapOriginY) / mapResolution;  
      if(checkNearObsReal(cellYObs-up, cellXObs-left)) {
        printf("laserHandle: runAgain: false\n");
        flag = 1;
        return;   
      }

      
      /*tu obi_i di sang trai va phai de tim ra gia tri left_i, right_i*/

      // tim right_i
      for(i = obs_i; i >= 0; i--) {
        if(i == 0) {right_i = i;break;}
        distance1 = std::fabs(laser->ranges[i] - laser->ranges[i-1]);
        distance2 = std::fabs(laser->ranges[i] - laser->ranges[i+1]);
        distance = std::fabs(distance1-distance2);
        if(distance > 0.1) {right_i = i;break;}
      }

      // tim left_i
      for(i = obs_i; i < laser->ranges.size(); i++) {
        if(i == (laser->ranges.size()-1)) {left_i = i;break;}
        distance1 = std::fabs(laser->ranges[i] - laser->ranges[i-1]);
        distance2 = std::fabs(laser->ranges[i] - laser->ranges[i+1]);
        distance = std::fabs(distance1-distance2);
        if(distance > 0.1) {left_i = i;break;}
      }

      for(i = right_i; i <= left_i; i++) {
        if(i < (laser->ranges.size() / 2)) {
          anpha = current_pose.theta - angleBetween2Ray*(laser->ranges.size()/2-i-0.5);
        } else {
          anpha = current_pose.theta + angleBetween2Ray*(i-laser->ranges.size()/2+0.5);
        }

        // xac dinh vi tri vat can
        amclPoseXObs = amclPoseX + (laser->ranges[i])*(std::cos(anpha));
        amclPoseYObs = amclPoseY + (laser->ranges[i])*(std::sin(anpha));
        cellXObs = (amclPoseXObs - mapOriginX) / mapResolution;
        cellYObs = mapHeight-1-(amclPoseYObs - mapOriginY) / mapResolution;

        if(!checkNearObsReal(cellYObs-up, cellXObs-left) &&
          cells[cellYObs-up][cellXObs-left].visitedReal != 1) {
          cells[cellYObs - up][cellXObs - left].isObReal = 2;
          runAgain = true;
        }

      }
      printf("laserHandle: left_i: %d | obs_i: %d | right_i: %d\n", left_i, obs_i, right_i);
      printf("laserHandle: runAgain: %d\n", runAgain);
    }
    flag = 1;     
  }
}

void KobukiController::amclHandle(const geometry_msgs::PoseWithCovarianceStampedConstPtr amclpose) {
  // Publish vi tri robot len rviz de view.
  // amcl_pub.publish(amclpose);

  /* Tu tin hieu laser tra ve, tin hieu se duoc xu ly qua thuat toan AMCL
  output cua AMCL la vi tri cua robot (pixel thu bao nhieu) 
  Tu output cua AMCL co the tinh duoc robot dang dung o cell thu bao nhieu (currentCellX, currentCellY)
  Chu y: currentCellX, currentCellY van tinh theo ban do file anh .pgm (ban do to, chua co up, left, down, right)
  */
  amclPoseX = amclpose->pose.pose.position.x;
  amclPoseY = amclpose->pose.pose.position.y;
  currentCellX = (amclpose->pose.pose.position.x - mapOriginX) / mapResolution;
  currentCellY = mapHeight-1-(amclpose->pose.pose.position.y - mapOriginY) / mapResolution;
  currentDirectionByAMCL = amclpose->pose.pose.orientation.z;
  // printf("currentCell: row %d col %d\n", currentCellY-up, currentCellX-left);
}

void KobukiController::initializeMap(const nav_msgs::MapMetaDataConstPtr msg) {
  if (initMap) {
    mapHeight = msg->height;
    mapWidth = msg->width;
    mapResolution = msg->resolution;
    cellSize = /*msg->resolution;*/0.125;
    mapOriginX = msg->origin.position.x;
    mapOriginY = msg->origin.position.y;

    map_sub = nh.subscribe("/map", 10, &KobukiController::setMapData, this);
    initMap = false;
  }
}

void KobukiController::setMapData(const nav_msgs::OccupancyGridConstPtr msg) {

  for (int i = 0; i < msg->data.size(); i++) {
    mapData.push_back((int) msg->data[i]);
  }

  ROS_INFO("INIT CELLS");
  // usleep(500000);
  mapArr = oneArrToTwoArr(mapData, mapWidth, mapHeight);
  up = findLineUp(mapArr, mapWidth, mapHeight);
  left = findLineLeft(mapArr, mapWidth, mapHeight);
  down = findLineDown(mapArr, mapWidth, mapHeight);
  right = findLineRight(mapArr, mapWidth, mapHeight);

  /*vi la quet den tung pixel anh mot nen mang cells se co nhieu vi tri lenh nhau 1 2 pixel, 
  ma thuat toan CCD bao phu tung pixel mot, dieu nay la khong tot. Nen la chung ta tuy chinh 
  up, left, right, down cho phu hop hon voi thuat toan */
  // voi ban do anhlaai_off
  // up++;down--;left++;right--;
  printf("up,left,down,right: %d %d %d %d\n", up, left, down, right);


  /* Khoi tao mang cells tu mang mapArr */
  int i,j;

  rows = down - up + 1;
  cols = right - left + 1;
  cells = (cell **)malloc(rows * sizeof(cell *));
  for (i = 0; i < rows; i++)
    cells[i] = (cell *)malloc(cols * sizeof(cell));
  
  for(i = 0; i < rows; i++) {
    for(j = 0; j < cols; j++) {
      // cells[i][j].g=10000.0;
      cells[i][j].row = i;
      cells[i][j].col = j;
      // cells[i][j].visitedDStar=0;
      // cells[i][j].visited=0;
      // cells[i][j].visitedReal=0;
      // cells[i][j].belongCcdPath = 0;
      if(i == 0 || j == 0 || i == rows-1 || j == cols-1) cells[i][j].isObReal = 1; 
      else cells[i][j].isObReal = 0;
    }
  }

  i=0;
  for(int row=up; row<=down; row++) {
    j=0;
    for(int col=left; col<=right; col++) {
      if(mapArr[row][col] != 0) cells[i][j].isObReal = 1; 
      j++;
    }
    i++;
  }

  // cells[11][32].isObReal = 0;
  // cells[21][27].isObReal = 0;
  // cells[22][27].isObReal = 0;
  // cells[23][27].isObReal = 0;
  // cells[24][27].isObReal = 0;
  // cells[25][27].isObReal = 0;
  // cells[26][27].isObReal = 0;
  // cells[27][27].isObReal = 0;
  // cells[28][27].isObReal = 0;

  // cells[29][22].isObReal = 1;
  // cells[30][14].isObReal = 0;

  // cells[43][22].isObReal = 0;
  // cells[43][21].isObReal = 0;
  // cells[43][20].isObReal = 0;

  cells[39][20].isObReal = 1;
  cells[39][21].isObReal = 1;
  cells[39][22].isObReal = 1;
  cells[39][23].isObReal = 1;
  cells[39][24].isObReal = 1;
  cells[39][25].isObReal = 1;
  cells[39][26].isObReal = 1;
  cells[39][27].isObReal = 1;

  cells[41][19].isObReal = 1;
  cells[42][19].isObReal = 1;
  cells[43][19].isObReal = 1;
  cells[44][19].isObReal = 1;
  cells[45][19].isObReal = 1;
  cells[46][19].isObReal = 1;
  cells[47][19].isObReal = 1;
  cells[48][19].isObReal = 1;
  cells[48][18].isObReal = 1;

  cells[36][7].isObReal = 1;
  cells[36][8].isObReal = 1;
  cells[36][9].isObReal = 1;
  cells[36][10].isObReal = 1;



  // in ban do cells de xem viec convert mapArr -> cells dung khong ?
  for(i=0;i<rows;i++) {
    for(j=0;j<cols;j++) {
      if(cells[i][j].isObReal == 1) printf("x ");
      else printf("  ");
    }
    printf("\n");
  }

  /* printMapWithCCD duoc them vao trong qua trinh lam
  de kiem tra do chinh xac cua AMCL thoi. Khong co y nghia trong giai thuat*/
  thread.start(&KobukiController::printMapWithCCD, *this);
  moveWithCCD();
}

/* doc chu thich trong ham setMapData */
void KobukiController::printMapWithCCD() {
  int x = 0,y = 0;
  int i,j;
  while(1) {
    while((x != currentCellX || y != currentCellY)) {
      system("clear");
      for(i=0;i<45;i++) {
        for(j=0;j<cols;j++) {
          if(cells[i][j].isObReal!=0) printf("* ");
          else if((currentCellY-up)==i && (currentCellX-left)==j) {
            printf("X ");
          }
          else if(cells[i][j].belongCcdPath == 1) printf(". ");
          else if(cells[i][j].visitedReal == 1) printf("O ");
          else printf("  ");
        }
        printf("\n");
      }
      x = currentCellX;
      y = currentCellY;
    }
  }
}

// setup tat ca cell nam trong pham vi robot bao phu la visited
void KobukiController::setVisitedReal(int row, int col) {
  int m, n;
  for(m = row-RM; m <= row+RM; m++) 
    for(n = col-RM; n <= col+RM; n++) 
      cells[m][n].visitedReal = 1;
}

void KobukiController::genePathFromCellToCell(int rowBegin, int colBegin, int rowEnd, int colEnd) {
  if(rowBegin == rowEnd && colBegin == colEnd) return;
  int r,c,i,j;
  queue q;
  cell ce;

  Init(&q);

  for(i = 0; i < rows; i++)
    for(j = 0; j < cols; j++) {
      cells[i][j].visitedDStar = 0;
    }

  r = rowBegin;
  c = colBegin;
  cells[r][c].bpRowv2 = r;
  cells[r][c].bpColv2 = c;
  cells[r][c].visitedDStar = 1;
  PushAndSortv2(&q, cells[r][c]);
  while(1) {
    ce = Pop(&q);
    r = ce.row;
    c = ce.col;
    if(cells[r+1][c].visitedDStar == 0 && cells[r+1][c].isObExtend != 1) {
      cells[r+1][c].gv2 = cells[r][c].gv2+1;
      cells[r+1][c].bpRowv2 = r;
      cells[r+1][c].bpColv2 = c;
      if(r+1==rowEnd && c==colEnd) return;
      cells[r+1][c].visitedDStar = 1;
      PushAndSortv2(&q, cells[r+1][c]);
    }
    if(cells[r-1][c].visitedDStar == 0 && cells[r-1][c].isObExtend != 1) {
      cells[r-1][c].gv2 = cells[r][c].gv2+1;
      cells[r-1][c].bpRowv2 = r;
      cells[r-1][c].bpColv2 = c;
      if(r-1==rowEnd && c==colEnd) return;
      cells[r-1][c].visitedDStar = 1;
      PushAndSortv2(&q, cells[r-1][c]);
    }
    if(cells[r][c+1].visitedDStar == 0 && cells[r][c+1].isObExtend != 1) {
      cells[r][c+1].gv2 = cells[r][c].gv2+1;
      cells[r][c+1].bpRowv2 = r;
      cells[r][c+1].bpColv2 = c;
      if(r==rowEnd && c+1==colEnd) return;
      cells[r][c+1].visitedDStar = 1;
      PushAndSortv2(&q, cells[r][c+1]);
    }
    if(cells[r][c-1].visitedDStar == 0 && cells[r][c-1].isObExtend != 1) {
      cells[r][c-1].gv2 = cells[r][c].gv2+1;
      cells[r][c-1].bpRowv2 = r;
      cells[r][c-1].bpColv2 = c;
      if(r==rowEnd && c-1==colEnd) return;
      cells[r][c-1].visitedDStar = 1;
      PushAndSortv2(&q, cells[r][c-1]);
    }
    if(cells[r+1][c+1].visitedDStar == 0 && cells[r+1][c+1].isObExtend != 1) {
      cells[r+1][c+1].gv2 = cells[r][c].gv2+1;
      cells[r+1][c+1].bpRowv2 = r;
      cells[r+1][c+1].bpColv2 = c;
      if(r+1==rowEnd && c+1==colEnd) return;
      cells[r+1][c+1].visitedDStar = 1;
      PushAndSortv2(&q, cells[r+1][c+1]);
    }
    if(cells[r+1][c-1].visitedDStar == 0 && cells[r+1][c-1].isObExtend != 1) {
      cells[r+1][c-1].gv2 = cells[r][c].gv2+1;
      cells[r+1][c-1].bpRowv2 = r;
      cells[r+1][c-1].bpColv2 = c;
      if(r+1==rowEnd && c-1==colEnd) return;
      cells[r+1][c-1].visitedDStar = 1;
      PushAndSortv2(&q, cells[r+1][c-1]);
    }
    if(cells[r-1][c+1].visitedDStar == 0 && cells[r-1][c+1].isObExtend != 1) {
      cells[r-1][c+1].gv2 = cells[r][c].gv2+1;
      cells[r-1][c+1].bpRowv2 = r;
      cells[r-1][c+1].bpColv2 = c;
      if(r-1==rowEnd && c+1==colEnd) return;
      cells[r-1][c+1].visitedDStar = 1;
      PushAndSortv2(&q, cells[r-1][c+1]);
    }
    if(cells[r-1][c-1].visitedDStar == 0 && cells[r-1][c-1].isObExtend != 1) {
      cells[r-1][c-1].gv2 = cells[r][c].gv2+1;
      cells[r-1][c-1].bpRowv2 = r;
      cells[r-1][c-1].bpColv2 = c;
      if(r-1==rowEnd && c-1==colEnd) return;
      cells[r-1][c-1].visitedDStar = 1;
      PushAndSortv2(&q, cells[r-1][c-1]);
    }


  }
}

int KobukiController::checkStraight(int row1, int col1, int row2, int col2, int row3, int col3) {
  if(row1 == row2 && row1 == row3) return 1;
  if(col1 == col2 && col1 == col3) return 1;
  float an1, an2;
  an1 = ((float)row1-(float)row2)/((float)col1-(float)col2);
  an2 = ((float)row1-(float)row3)/((float)col1-(float)col3);
  if(an1 == an2) return 1;
  return 0;
}


void KobukiController::moveWithCCD() {
  // turn(PI/2);
  // go(1);
  // while(ros::ok()) {

  // }
  // return;




  int firstRow, firstCol;
  queue ccdPathQueue, ccdPathQueueOld;
  node *temp, *temp2, *temp3, *temp4;
  int i,j;
  float angle, dis;

  ROS_INFO("START MOVE CCD");

  // su dung AMCL lay vi tri ban dau
  printf("Determining current location by AMCL\n");
  turn(-PI/2);
  firstRow = currentCellY - up;  
  firstCol = currentCellX - left; 
  printf("firstRow: %d | firstCow: %d\n", firstRow, firstCol);
  
  setRowsAndCols(rows, cols);

  // thuat toan CCD* 
  buildObExtend(cells);
  dstar(cells, firstRow, firstCol);
  ccdPathQueue = coverage(cells, firstRow, firstCol);

  temp = ccdPathQueue.Front;
  temp3 = temp;
  
  while(temp != ccdPathQueue.Rear) {
    temp2 = temp;
    temp = temp->Next;

    if(checkSpecialPoint(temp, temp2, ccdPathQueue)) {
      // angle = getAngleFromCellToCell(currentCellY - up,currentCellX - left, (temp->Data).row, (temp->Data).col);
      
      turnToCell((temp->Data).row, (temp->Data).col);
      dis = getDistanceFromCellToCell(currentCellY - up,currentCellX - left,(temp->Data).row, (temp->Data).col, cellSize);
      go(dis);

      /* doi laser xu ly du lieu xong */
      while(flag == 0) {}

      if(!runAgain) {
        temp4 = temp3;
        setVisitedReal((temp4->Data).row, (temp4->Data).col);
        while(temp4 != temp) {
          temp4 = temp4->Next;
          setVisitedReal((temp4->Data).row, (temp4->Data).col);
        }

        temp3 = temp;
        isObs = false;
      } else {

        while(1) {
          printf("BEGIN run again CCD*\n");
          for(i=0;i<rows;i++) 
            for(j=0;j<cols;j++) {
              cells[i][j].visited = 0;
              cells[i][j].belongCcdPath = 0;
            }

          buildObExtend(cells);
          dstar(cells, firstRow, firstCol);
          ccdPathQueueOld = ccdPathQueue;
          ccdPathQueue = coverage(cells, firstRow, firstCol);


          // in ban do

          for(i=0;i<45;i++) {
            for(j=0;j<cols;j++) {
              if(cells[i][j].isObReal!=0) printf("* ");
              else if(cells[i][j].belongCcdPath == 1) printf(". ");
              // else if(cells[i][j].visitedReal == 1) printf("0 ");
              else printf("  ");
            }
            printf("\n");
          }

// printf("a\n");
          temp = findLastCommonPoint(&ccdPathQueueOld, &ccdPathQueue);
          temp4 = temp3;
          // printf("b\n");
          setVisitedReal((temp4->Data).row, (temp4->Data).col);
          // printf("c\n");
          // while(temp4 != temp) {
          //   printf("d\n");
          //   temp4 = temp4->Next;
          //   setVisitedReal((temp4->Data).row, (temp4->Data).col);
          // }
          temp3 = temp;
          printf("END run again CCD*\n");
          runAgain = false;
          isObs = false;

          printf("moveWithCCD: go back to temp3\n");
          printf("%d %d %d %d\n", (temp3->Data).row,(temp3->Data).col, currentCellY - up,currentCellX - left);
          cells[currentCellY - up][currentCellX - left].isObExtend = 0;
          genePathFromCellToCell((temp3->Data).row,(temp3->Data).col, currentCellY - up,currentCellX - left);
          int x, y, m, n, h, k;
          float an1,an2;
          y = currentCellY - up;
          x = currentCellX - left;

          if(!(y==(temp3->Data).row && x==(temp3->Data).col)) {
            while(1) {
              n = cells[y][x].bpRowv2;
              m = cells[y][x].bpColv2;
              h = cells[n][m].bpRowv2;
              k = cells[n][m].bpColv2;
              if(!checkStraight(y,x,n,m,k,h) || (n==(temp3->Data).row && m==(temp3->Data).col)) {
                // angle = getAngleFromCellToCell(currentCellY - up,currentCellX - left,n,m);
                
                turnToCell(n,m);
                dis = getDistanceFromCellToCell(currentCellY - up,currentCellX - left,n,m,cellSize);
                go(dis);
                /* doi laser xu ly du lieu xong */
                while(flag == 0) {}
                if(runAgain) break;
                if((n==(temp3->Data).row && m==(temp3->Data).col)) break;
              }
              y=n;
              x=m;
            }
          }

          if(!runAgain) break;
        }
      }
    }

  }
  
  ROS_INFO("END MOVE CCD");

}
