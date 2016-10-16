/*
 *    Copyright (C) 2016 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{



	
	timer.start(Period);
	

	return true;
}

void SpecificWorker::compute()
{   

   const float threshold = 400; //millimeters
     float rot = 0.5;  //rads per second

    try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
        std::sort( ldata.begin()+5, ldata.end()-5, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.

        if(target.active==true){

//     if( ldata[6].dist < threshold)
//     {
//       if(ldata[6].angle > 0){
//         std::cout << ldata.front().dist << std::endl;
//         differentialrobot_proxy->setSpeedBase(5, -rot);
//         usleep(rand()%(1500000-100000 + 1) + 100000);  //random wait between 1.5s and 0.1sec
//       }else{
// 	std::cout << ldata.front().dist << std::endl;
//         differentialrobot_proxy->setSpeedBase(5, rot);
//         usleep(rand()%(1500000-100000 + 1) + 100000);  //random wait between 1.5s and 0.1sec
//       }
//       
//     }   
//     else
//     {
//         differentialrobot_proxy->setSpeedBase(200, 0); 
//     }
//	}
 
  
//  if(target.active==true){
    float x,z,a,modulo,angulo;
    QVec poseAux;

    
    RoboCompDifferentialRobot::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
    x=bState.x;
    z=bState.z;
    a=bState.alpha;
    
   poseAux= target.getPose();
   
   poseAux[0] = poseAux.x()-bState.x;
   poseAux[1] = poseAux.z()-bState.z;
   
   modulo=sqrt((poseAux.x()*poseAux.x())+(poseAux.z()*poseAux.z()));

    
    float R [2][2];
    R[0][0]=cos(a);
    R[0][1]=-sin(a);
    R[1][0]=sin(a);
    R[1][1]=cos(a);
    
    float T [2];
    T[0]=poseAux.x();
    T[1]=poseAux.z();
    
    float W [2];
    T[0]=x;
    T[1]=z;
    
    float Y[2];
    Y[0]= ((R[0][0]*W[0]) + (R[0][1]*W[1])) + T[0];
    Y[1]= ((R[1][0]*W[0]) + (R[1][1]*W[1])) + T[1];
    
    angulo=atan2(Y[0],Y[1]);
    
     if( ldata[6].dist < threshold)
    {
      if(ldata[6].angle > 0){
	std::cout << ldata.front().dist << std::endl;
	differentialrobot_proxy->setSpeedBase(modulo,angulo);  
	usleep(rand()%(1500000-100000 + 1) + 100000);  //random wait between 1.5s and 0.1sec
    
      }else{
	std::cout << ldata.front().dist << std::endl;
        differentialrobot_proxy->setSpeedBase(modulo, -angulo);
        usleep(rand()%(1500000-100000 + 1) + 100000);  //random wait between 1.5s and 0.1sec
      }
    }else
    {
        differentialrobot_proxy->setSpeedBase(200, 0); 
    }
     
   }
     }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
 

}

//////////////////////////////////////////////////////////////
void SpecificWorker::setPick(const Pick& myPick)
{
  target.copy( myPick.x, myPick.z);
  target.setActive(this);
  qDebug() << myPick.x<<myPick.z;
}









