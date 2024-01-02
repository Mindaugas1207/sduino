













    //auto [ENC_Data, ENC_OK] = getEncoderData();
    //auto [Roll, Pitch, Yaw] = IMU0.GetOrientation();

    //float SpeedR, SpeedL;

    // if (ENC_OK)
    // {
    //     //Calculate motors speeds, -1.0f to 1.0f relative to MOTOR_NOMINAL_RPM, valuse below MOTOR_MIN_RPM are set to 0.0f, above MOTOR_MAX_RPM to 1.0f
    //     int PPA = std::min(ENC_Data.PulsePeriodSumA, ENCODER_SAMPLE_BUFFER_SIZE * (int)ENCODER_MAX_PULSE_PERIOD_US);
    //     int PPB = std::min(ENC_Data.PulsePeriodSumB, ENCODER_SAMPLE_BUFFER_SIZE * (int)ENCODER_MAX_PULSE_PERIOD_US);
    //     SpeedR = (((float)ENCODER_SAMPLE_BUFFER_SIZE - ENCODER_CONVERSION_BOT * PPA) / (PPA * ENCODER_CONVERSION_SPAN)) * ENC_Data.PulseDirectionA;
    //     SpeedL = (((float)ENCODER_SAMPLE_BUFFER_SIZE - ENCODER_CONVERSION_BOT * PPB) / (PPB * ENCODER_CONVERSION_SPAN)) * ENC_Data.PulseDirectionB;  
    //     iSpeedR = SpeedR;
    //     iSpeedL = SpeedL;

    //     int PulseR = -ENC_Data.PulseCountA; //invert right motor fedback
    //     int PulseL = ENC_Data.PulseCountB;

    //     auto [dX, dY, dYaw] = XYoffsetEncoders(PulseR - iPulseR, PulseL - iPulseL, iYaw);
    //     step_counterA += PulseR - iPulseR;
    //     step_counterB += PulseL - iPulseL;
    //     iPulseR = PulseR;
    //     iPulseL = PulseL;

//1 0 0 1 0 0 1 0 0 1
//0.5,0.25,0.125,

    //     iX_f += dX;
    //     iY_f += dY;
    //     iYaw = constrainAngle(iYaw + dYaw);

    //     // auto flrX = std::floor(iX_f);
    //     // auto flrY = std::floor(iY_f);

    //     // if (flrX != 0.0f)
    //     // {
    //     //     iX += flrX;
    //     //     iX_f -= flrX;
    //     // }

    //     // if (flrY != 0.0f)
    //     // {
    //     //     iY += flrY;
    //     //     iY_f -= flrY;
    //     // }
    // }
    // else
    // {
    //     SpeedR = iSpeedR;
    //     SpeedL = iSpeedL;
    // }

    // if (!doManuver && !manuver_done && (Start && !Stop))
    // {
    //     if (vlx_start)
    //     {
    //         if (vlx_measurement > 700)
    //         {
    //             vlx_start = false;
    //             vlxTime = get_absolute_time();
    //         }
    //         else if (to_ms_since_boot(vlxTime) - to_ms_since_boot(get_absolute_time()) > 20)
    //         {
    //             vlx_start = false;
    //             doManuver = true;
    //             manuverStep = 0;
    //             PathTime = get_absolute_time();
    //         }
    //     }
    //     else
    //     {
    //         if (vlx_measurement < 700)
    //         {
    //             vlx_start = true;
    //             vlxTime = get_absolute_time();
    //         }
    //     }
    // }
    

    //auto LineHeading = LineSensor.computeLineHeading(Yaw);
    //auto PathHeading = computePathHeading(Yaw, !doManuver);
    //auto [Range, Turn] = LineSensor.getLineDesc();
    //auto Detected = LineSensor.isDetected();

    // if (to_us_since_boot(get_absolute_time()) > to_us_since_boot(PrintTime))
    // {
    // //     //printf("LH: %f \n", LineHeading);
    // //     //printf("A: %f, B: %f\n", DriveA.getDuty(), DriveB.getDuty());
    // //     //printf("M1 %d, % .3f rpm, M2 %d, % .3f rpm, % .9f\n", Steps1, RPM1, Steps2, RPM2, PID_DriveB.getIntg());
    //     // char buffer[256];
    //     // //sprintf(buffer, "CMD:WS(\"R\":%.2f,\"L\":%.2f,\"Y\":%.3f,\"H\":%.3f)\n", SpeedR, SpeedL, Yaw, LineHeading);

    //     // int n = sprintf(buffer, "CMD:WS(");

    //     // for (int i = 0; i < LineSensor._LineCount; i++)
    //     // {
    //     //     n += sprintf(buffer + n, "[%d,%d,%c]\n", LineSensor._Lines[i].Start, LineSensor._Lines[i].End, LineSensor._Lines[i].Color ? 'B' : 'W');
    //     // }
    //     // n += sprintf(buffer, ")\n");
    //     // uart_puts(uart_internal, buffer);
    // //     // char str1[15];
    // //     // char str2[15];
    // //     char buffer[128];
    // //     //sprintf(str1, "%.9f", iX_f);
    // //     //sprintf(str2, "%.9f", iY_f);
    // //     //sprintf(buffer, "CMD:WS(\"X\":%d.%s,\"Y\":%d.%s,\"O\":%.9f,\"L\":%.9f,\"I\":%.9f)\n", iX, str1 + 2, iY, str2 + 2, iYaw, LineHeading, Yaw);
    // //     sprintf(buffer, "CMD:WS(\"X\":%.12f,\"Y\":%.12f,\"O\":%.9f,\"L\":%.9f,\"I\":%.9f)\n", iX_f, iY_f, iYaw, LineHeading, Yaw);
    // //     uart_puts(uart_internal, buffer);
    // //     //printf("R: %d, L: %d, X: %d %f Y: %d %f O: %.9f L: %.9f Yaw: %.9f\n", iPulseR, iPulseL, iX, iX_f, iY, iY_f, iYaw, LineHeading, Yaw);
    // //     //printf("sB:%f sA:%f BS:%f, SO:%f, oB:%f, oA:%f, vB:%f, vA:%f, M+:%f, M-:%f, iB:%f, iA:%f, pB:%f, pA:%f\n", SpeedA, SpeedB, BaseSpeed, SteeringOffset, OffsetB, OffsetA, OverdriveB, OverdriveA, OffsetMaxPos, OffsetMaxNeg, PID_DriveB.getIntegral(), PID_DriveA.getIntegral(), DriveB.getDuty(), DriveA.getDuty());
    //      PrintTime = make_timeout_time_ms(250);
    // }
    
    //EncoderPulseCountA = ENC_Data.PulseCountA;
    //EncoderPulseCountB = ENC_Data.PulseCountB;
    //auto [Steps1, Steps2] = getSteps();
    
    // if (to_us_since_boot(get_absolute_time()) > to_us_since_boot(PrintTime))
    // {
    //     printf("Detected: %s, Heading: %.6f, Range: %s, Turn: %s\n",
    //         Detected ? "True " : "False",
    //         LineHeading, 
    //         Range == LineSensor_s::LineRange_t::LINE_CENTER ? "CENTER" :
    //         Range == LineSensor_s::LineRange_t::LINE_LEFT ? "LEFT  " : "RIGHT ",
    //         Turn == LineSensor_s::LineTurn_t::LINE_NO_TURN ? "NONE  " :
    //         Turn == LineSensor_s::LineTurn_t::LINE_TURN_LEFT ? "LEFT  " : "RIGHT "
    //     );
    //     PrintTime = make_timeout_time_ms(200);
    // }

   // float SPA, SPB;
    //float BaseSpeed, SteeringOffset, SteeringGain, CruiseGain;

    // if (Range == LineSensor_s::LineRange_t::LINE_CENTER)
    // {
    //     ReturningOnLine = false;
    // }

    //bool LineTracking = (Detected && !ReturningOnLine);// || (!Detected && Turn != LineSensor_s::LineTurn_t::LINE_NO_TURN);
    //bool LineTracking = Detected;

    // if (doManuver)
    // {
    //     float dest = 0.0f;
    //     float dist_mm = (step_counterA + step_counterB * ENCODER_PULSE_TO_LENGTH_MM) / 2;
    //     switch(manuverStep)
    //     {
    //         case 0:
    //         dest = LINE_SENSOR_TURN90_VALUE;
    //         if (std::abs(PathHeading) < LINE_SENSOR_STEP_VALUE / 10.0f)
    //         {
    //             manuverStep++;
    //             step_counterA = 0;
    //             step_counterB = 0;
    //             PathTime = get_absolute_time();
    //             break;
    //         }
    //         break;
    //         case 1:
    //         dest = LINE_SENSOR_TURN90_VALUE;
    //         if (dist_mm > 500)
    //         {
    //             manuverStep++;
    //             PathTime = get_absolute_time();
    //             break;
    //         }
    //         break;
    //         case 2:
    //         dest = 0.0f;
    //         if (std::abs(PathHeading) < LINE_SENSOR_STEP_VALUE / 10.0f)
    //         {
    //             manuverStep++;
    //             step_counterA = 0;
    //             step_counterB = 0;
    //             PathTime = get_absolute_time();
    //             break;
    //         }
    //         break;
    //         case 3:
    //         dest = 0.0f;
            
    //         if (dist_mm > 500)
    //         {
    //             manuverStep++;
    //             PathTime = get_absolute_time();
    //             break;
    //         }
    //         break;
    //         case 4:
    //         dest = -LINE_SENSOR_TURN90_VALUE;
    //         if (std::abs(PathHeading) < LINE_SENSOR_STEP_VALUE / 10.0f)
    //         {
    //             manuverStep++;
    //             step_counterA = 0;
    //             step_counterB = 0;
    //             PathTime = get_absolute_time();
    //             break;
    //         }
    //         break;
    //         case 5:
    //         default:
    //         dest = -LINE_SENSOR_TURN90_VALUE;
    //         if (Detected || to_ms_since_boot(PathTime) - to_ms_since_boot(get_absolute_time()) > 5000)
    //         {
    //             doManuver = false;
    //             manuver_done = true;
    //             manuverStep = 0;
    //         }
    //         break;
    //     }
        
    //     SteeringGain = Config.SteeringGain4;
    //     CruiseGain = Config.CruiseGain4;
    //     PathHeading = dest - PathHeading;
    //     SteeringOffset = -PathHeading * SteeringGain;

    //     BaseSpeed = Config.ForwardSpeed - std::clamp(std::abs(PathHeading) * CruiseGain, 0.0f, Config.ForwardSpeed);

    //     SPA = std::clamp(SteeringOffset >= 0.0f ? BaseSpeed : BaseSpeed + SteeringOffset, -10.0f, 10.0f);
    //     SPB = std::clamp(SteeringOffset <= 0.0f ? BaseSpeed : BaseSpeed - SteeringOffset, -10.0f, 10.0f);
    // }
    // else
    // {
        // if (LineTracking)
        // {
        //     BrakeStart = false;
        //     BrakeEnd = false;
        //     SteeringGain = Config.SteeringGain;
        //     CruiseGain = Config.CruiseGain;
        //     LineHeading = std::abs(LineHeading) < LINE_SENSOR_STEP_VALUE / 10.0f ? 0.0f : LineHeading;
        //     PID_Tracking.setSetPoint(0.0f);
        //     SteeringOffset = PID_Tracking.compute(LineHeading, _TimeNow);

        //     BaseSpeed = Config.ForwardSpeed - std::clamp(std::abs(LineHeading) * CruiseGain, 0.0f, Config.ForwardSpeed);

        //     SPA = std::clamp(SteeringOffset >= 0.0f ? BaseSpeed : BaseSpeed + SteeringOffset, -10.0f, 10.0f);
        //     SPB = std::clamp(SteeringOffset <= 0.0f ? BaseSpeed : BaseSpeed - SteeringOffset, -10.0f, 10.0f);
        // }
        // else
        // {
        //     ReturningOnLine = true;
        //     if (Config.BrakeTimeout != 0)
        //     {
        //         if (!BrakeStart)
        //         {
        //             BrakeStart = true;
        //             BrakeTime = make_timeout_time_us(Config.BrakeTimeout);
        //         }
        //     }

        //     float SteeringGainF = Config.SteeringGain2;
        //     float SteeringGainB = Config.SteeringGain3;
        //     CruiseGain = Config.CruiseGain2;
        //     BaseSpeed = Config.ForwardSpeed - std::clamp(std::abs(LineHeading) * CruiseGain, 0.0f, Config.ForwardSpeed);
        //     float SteeringOffsetF = -LineHeading * SteeringGainF;
        //     float SteeringOffsetB = -LineHeading * SteeringGainB;

        //     if (LineHeading > 0)
        //     {
        //         SPA = std::clamp(BaseSpeed + SteeringOffsetB, -10.0f, 10.0f);
        //         SPB = std::clamp(BaseSpeed - SteeringOffsetF, -10.0f, 10.0f);
        //     }
        //     else
        //     {
        //         SPA = std::clamp(BaseSpeed + SteeringOffsetF, -10.0f, 10.0f);
        //         SPB = std::clamp(BaseSpeed - SteeringOffsetB, -10.0f, 10.0f);
        //     }
        // }
    //}

    

    // if (!Start || Stop)
    // {
    //     if (to_us_since_boot(_TimeNow) > to_us_since_boot(SleepTime) && Awake && SleepAllowed)
    //     {
    //         sleep();
    //     }
    //     return;
    // }
    // else if (Start && !Stop)
    // {
    //     // if (Roll > 0.6f || Roll < -0.6f || Pitch > 0.6f || Pitch < -0.6f)
    //     // {
    //     //     if (!Stop)
    //     //         stop_error();
    //     //     return;
    //     // }

    //     // if (LineSensor.isTimedOut())
    //     // {
    //     //     if (!Stop)
    //     //         stop_error();
    //     //     return;
    //     // }

    //     if (BrakeStart && !BrakeEnd)
    //     {
    //         if (to_us_since_boot(_TimeNow) < to_us_since_boot(BrakeTime))
    //         {
    //             DriversBrake();
    //         }
    //         else
    //         {
    //             DriversStart();
    //             BrakeEnd = true;
    //         }
    //     }

    //     if (!BrakeStart || BrakeEnd)
    //     {
    //         PID_DriveA.setSetPoint(-SPA);
    //         PID_DriveB.setSetPoint(SPB);

    //         MotorDriverA.SetPower(PID_DriveA.compute(SpeedR, _TimeNow)); //invert rigth motor feedback
    //         MotorDriverB.SetPower(PID_DriveB.compute(SpeedL, _TimeNow));
    //     }









