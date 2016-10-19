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
 
  
  if(target.active==true){
    float x,z,a,modulo,angulo, valorAbsoluto;
    bool giro = true;
    
    //Obtenemos x,z alpha del robot
    RoboCompDifferentialRobot::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
    x=bState.x;
    z=bState.z;
    a=bState.alpha;
    
   //Calculamos distancia entre coordenadas del robot y el click
   float distancia[2];
   distancia[0] = (target.getPose().x() - x);
   distancia[1] = (target.getPose().z() - z);
   modulo= sqrt((distancia[0]*distancia[0]) + (distancia[1]*distancia[1]));

    //Matriz senoidal
    float R [2][2];
    R[0][0]=cos(a);
    R[0][1]=-sin(a);
    R[1][0]=sin(a);
    R[1][1]=cos(a);
    
    //Calculamos el angulo entre los puntos
    float Y[2];
    Y[0]= (R[0][0]*distancia[0]) + (R[0][1]*distancia[1]);
    Y[1]= (R[1][0]*distancia[0]) + (R[1][1]*distancia[1]);
    angulo=atan2(Y[0],Y[1]);
    
    //Calculamos el valor absoluto del angulo
    valorAbsoluto=abs(angulo);
    
    if(giro){
      differentialrobot_proxy->setSpeedBase(200,0);
      if(modulo < threshold){
	differentialrobot_proxy->stopBase();
	giro =false;
	target.setActive(false);
      }
    }else{
      
      if(valorAbsoluto > rot)
    {
      differentialrobot_proxy->setSpeedBase(0,angulo);
     
    }else{
      differentialrobot_proxy->stopBase();
      giro = true;
    }
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









