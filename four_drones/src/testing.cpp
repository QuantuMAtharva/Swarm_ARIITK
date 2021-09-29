#include<iostream>
#include<vector>
#include<numeric>
#include<cmath>


float app_vel(std::vector<float>vec_pos1,std::vector<float>vec_outdrone_pos,std::vector<float>vec_vel1,std::vector<float>vec_outdrone_vel)
{
  float vel1,vel2;
  std::vector<float> vec_del_pos {vec_pos1[0]-vec_outdrone_pos[0] , vec_pos1[1]-vec_outdrone_pos[1] , vec_pos1[2]-vec_outdrone_pos[2]};
  if(vec_del_pos[0]<0.1 && vec_del_pos[1]<0.1 && vec_del_pos[2]<0.1)
  {
    vec_del_pos[0]=0.1;vec_del_pos[1]=0.1;vec_del_pos[2]=0.1;
  }
  vel1 = std::inner_product(vec_vel1.begin(), vec_vel1.end(), vec_del_pos.begin(), 0.0)/(pow(pow(vec_del_pos[0],2)+pow(vec_del_pos[1],2)+pow(vec_del_pos[2],2),0.5));
  vel2 = std::inner_product(vec_outdrone_vel.begin(), vec_outdrone_vel.end(), vec_del_pos.begin(), 0.0)/(pow(pow(vec_del_pos[0],2)+pow(vec_del_pos[1],2)+pow(vec_del_pos[2],2),0.5));
  
  if(vel1-vel2>0)
  return(0);

  return vel1-vel2;
}

int main()
{
    std::vector<float> v1 {1,2,3};
    std::vector<float> v2 {1,2,3};
    std::vector<float> v3 {5,6,2};
    std::vector<float> v4 {5,6,2};

    std::cout<<app_vel(v1,v2,v3,v4);
}