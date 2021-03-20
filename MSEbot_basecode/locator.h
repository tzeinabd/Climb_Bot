


void trackBeacon() //make sure beacon stays in sight
{
  LOC_btTrackingBeacon = true;
  ENC_SetDistance(2000, 2000);
  ucMotorState = 1;   //forward
  CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
  CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;

}

void findBeacon()
{
    CR1_ui8WheelSpeed = 140;
    LOC_btLookingForBeaconFlag = true;
    ENC_SetDistance(1000, 1000);
    
    if(LOC_ciLastTurnDirection ==2)
    {
      ucMotorState = 3; //right
      CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
      CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
    }
    else
    {
      ucMotorState = 2;  //left
      CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
      CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
    }

    LOC_ciLastTurnDirection = ucMotorState;
    
}

void beaconFound()
{
  LOC_btLookingForBeaconFlag = false;
  if(LOC_btTrackingBeacon)
  {
    trackBeacon();
  }
}
