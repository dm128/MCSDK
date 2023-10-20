
/**
  ******************************************************************************
  * @file    register_interface.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the register access for the MCP protocol
  *
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#include "mc_type.h"
#include "string.h"
#include "register_interface.h"
#include "mc_config.h"
#include "mcp.h"
#include "mcp_config.h"
#include "mcpa.h"
#include "dac_ui.h"
#include "mp_one_touch_tuning.h"
#include "mp_self_com_ctrl.h"
#include "mc_configuration_registers.h"

uint8_t RI_SetRegisterGlobal(uint16_t regID, uint8_t typeID, uint8_t *data, uint16_t *size, int16_t dataAvailable)
{
  uint8_t retVal = MCP_CMD_OK;
  switch(typeID)
  {
    case TYPE_DATA_8BIT:
    {
      switch (regID)
      {
        case MC_REG_STATUS:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }

        default:
        {
          retVal = MCP_ERROR_UNKNOWN_REG;
          break;
        }
      }
      *size = 1;
      break;
    }

    case TYPE_DATA_16BIT:
    {
      uint16_t regdata16 = *(uint16_t *)data; //cstat !MISRAC2012-Rule-11.3
      switch (regID)
      {

        case MC_REG_BUS_VOLTAGE:
        case MC_REG_HEATS_TEMP:
        case MC_REG_MOTOR_POWER:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }

        case MC_REG_DAC_OUT1:
        {
          DAC_SetChannelConfig(&DAC_Handle , DAC_CH1, regdata16);
          break;
        }

        case MC_REG_DAC_OUT2:
        {
          DAC_SetChannelConfig(&DAC_Handle , DAC_CH2, regdata16);
          break;
        }

        case MC_REG_DAC_USER1:
        case MC_REG_DAC_USER2:
        break;

        default:
        {
          retVal = MCP_ERROR_UNKNOWN_REG;
          break;
        }
      }
      *size = 2;
      break;
    }

    case TYPE_DATA_32BIT:
    {

      switch (regID)
      {
        case MC_REG_FAULTS_FLAGS:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }

        default:
        {
          retVal = MCP_ERROR_UNKNOWN_REG;
          break;
        }
      }
      *size = 4;
      break;
    }

    case TYPE_DATA_STRING:
    {
      const char_t *charData = (const char_t *)data;
      char_t *dummy = (char_t *)data;
      retVal = MCP_ERROR_RO_REG;
      /* Used to compute String length stored in RXBUFF even if Reg does not exist */
      /* It allows to jump to the next command in the buffer */
      (void)RI_MovString(charData, dummy, size, dataAvailable);
      break;
    }

    case TYPE_DATA_RAW:
    {
      uint16_t rawSize = *(uint16_t *)data; //cstat !MISRAC2012-Rule-11.3
      /* The size consumed by the structure is the structure size + 2 bytes used to store the size */
      *size = rawSize + 2U;
      uint8_t *rawData = data; /* rawData points to the first data (after size extraction) */
      rawData++;
      rawData++;

      if (*size > (uint16_t)dataAvailable)
      {
        /* The decoded size of the raw structure can not match with transmitted buffer, error in buffer
           construction */
        *size = 0;
        retVal = MCP_ERROR_BAD_RAW_FORMAT; /* This error stop the parsing of the CMD buffer */
      }
      else
      {
        switch (regID)
        {
          case MC_REG_APPLICATION_CONFIG:
          case MC_REG_MOTOR_CONFIG:
          case MC_REG_GLOBAL_CONFIG:
          case MC_REG_FOCFW_CONFIG:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }

          default:
          {
            retVal = MCP_ERROR_UNKNOWN_REG;
            break;
          }
        }

      }
      break;
    }

    default:
    {
      retVal = MCP_ERROR_BAD_DATA_TYPE;
      *size =0; /* From this point we are not able anymore to decode the RX buffer */
      break;
    }
  }
  return (retVal);
}

uint8_t RI_SetRegisterMotor1(uint16_t regID, uint8_t typeID, uint8_t *data, uint16_t *size, int16_t dataAvailable)
{
  uint8_t retVal = MCP_CMD_OK;
  uint8_t motorID=0;
  MCI_Handle_t *pMCIN = &Mci[motorID];

  switch(typeID)
  {
    case TYPE_DATA_8BIT:
    {
      switch (regID)
      {
        case MC_REG_STATUS:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }

        case MC_REG_CONTROL_MODE:
        {
          uint8_t regdata8 = *data;
          if ((uint8_t)MCM_TORQUE_MODE == regdata8)
          {
            MCI_ExecTorqueRamp(pMCIN, MCI_GetTeref(pMCIN), 0);
          }
          else
          {
            /* Nothing to do */
          }

          if ((uint8_t)MCM_SPEED_MODE == regdata8)
          {
            MCI_ExecSpeedRamp(pMCIN, MCI_GetMecSpeedRefUnit(pMCIN), 0);
          }
          else
          {
            /* Nothing to do */
          }

          break;
        }

        case MC_REG_RUC_STAGE_NBR:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }
        case MC_REG_SC_PP:
        {
          uint8_t regdataU8 = *(uint8_t *)data;
          SPD_SetElToMecRatio(&STO_PLL_M1._Super, regdataU8);
          SPD_SetElToMecRatio(&VirtualSpeedSensorM1._Super, regdataU8);
          SPD_SetElToMecRatio(&HALL_M1._Super, regdataU8);
          SCC_SetPolesPairs(&SCC, regdataU8);
          OTT_SetPolesPairs(&OTT, regdataU8);
          break;
        }

        case MC_REG_SC_CHECK:
        case MC_REG_SC_STATE:
        case MC_REG_SC_STEPS:
        case MC_REG_SC_FOC_REP_RATE:
        case MC_REG_SC_COMPLETED:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }

        case MC_REG_HT_MECH_WANTED_DIRECTION:
        {
          uint8_t regdataU8 = *(uint8_t *)data;
          HT_SetMechanicalWantedDirection(&HT,regdataU8);
          break;
        }

          case MC_REG_PB_CHARACTERIZATION:
        {
          uint8_t regdataU8 = *(uint8_t *)data;
          SCC_SetPBCharacterization(&SCC,regdataU8);
          break;
        }

        default:
        {
          retVal = MCP_ERROR_UNKNOWN_REG;
          break;
        }
      }
      *size = 1;
      break;
    }

    case TYPE_DATA_16BIT:
    {
      uint16_t regdata16 = *(uint16_t *)data; //cstat !MISRAC2012-Rule-11.3
      switch (regID)
      {
        case MC_REG_SPEED_KP:
        {
          PID_SetKP(&PIDSpeedHandle_M1, (int16_t)regdata16);
          break;
        }

        case MC_REG_SPEED_KI:
        {
          PID_SetKI(&PIDSpeedHandle_M1, (int16_t)regdata16);
          break;
        }

        case MC_REG_SPEED_KD:
        {
          PID_SetKD(&PIDSpeedHandle_M1, (int16_t)regdata16);
          break;
        }

        case MC_REG_I_Q_KP:
        {
          PID_SetKP(&PIDIqHandle_M1, (int16_t)regdata16);
          break;
        }

        case MC_REG_I_Q_KI:
        {
          PID_SetKI(&PIDIqHandle_M1, (int16_t)regdata16);
          break;
        }

        case MC_REG_I_Q_KD:
        {
          PID_SetKD(&PIDIqHandle_M1, (int16_t)regdata16);
          break;
        }

        case MC_REG_I_D_KP:
        {
          PID_SetKP(&PIDIdHandle_M1, (int16_t)regdata16);
          break;
        }

        case MC_REG_I_D_KI:
        {
          PID_SetKI(&PIDIdHandle_M1, (int16_t)regdata16);
          break;
        }

        case MC_REG_I_D_KD:
        {
          PID_SetKD(&PIDIdHandle_M1, (int16_t)regdata16);
          break;
        }

        case MC_REG_FLUXWK_KP:
        {
          PID_SetKP(&PIDFluxWeakeningHandle_M1, (int16_t)regdata16);
          break;
        }

        case MC_REG_FLUXWK_KI:
        {
          PID_SetKI(&PIDFluxWeakeningHandle_M1, (int16_t)regdata16);
          break;
        }

        case MC_REG_FLUXWK_BUS:
        {
          FW_SetVref(&FW_M1, regdata16);
          break;
        }

        case MC_REG_BUS_VOLTAGE:
        case MC_REG_HEATS_TEMP:
        case MC_REG_MOTOR_POWER:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }

        case MC_REG_I_A:
        case MC_REG_I_B:
        case MC_REG_I_ALPHA_MEAS:
        case MC_REG_I_BETA_MEAS:
        case MC_REG_I_Q_MEAS:
        case MC_REG_I_D_MEAS:
        case MC_REG_FLUXWK_BUS_MEAS:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }

        case MC_REG_I_Q_REF:
        {
          qd_t currComp;
          currComp = MCI_GetIqdref(pMCIN);
          currComp.q = (int16_t)regdata16;
          MCI_SetCurrentReferences(pMCIN,currComp);
          break;
        }

        case MC_REG_I_D_REF:
        {
          qd_t currComp;
          currComp = MCI_GetIqdref(pMCIN);
          currComp.d = (int16_t)regdata16;
          MCI_SetCurrentReferences(pMCIN,currComp);
          break;
        }

        case MC_REG_V_Q:
        case MC_REG_V_D:
        case MC_REG_V_ALPHA:
        case MC_REG_V_BETA:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }

        case MC_REG_HALL_EL_ANGLE:
        case MC_REG_HALL_SPEED:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }

        case MC_REG_STOPLL_C1:
        {
          int16_t hC1;
          int16_t hC2;
          STO_PLL_GetObserverGains(&STO_PLL_M1, &hC1, &hC2);
          STO_PLL_SetObserverGains(&STO_PLL_M1, (int16_t)regdata16, hC2);
          break;
        }

        case MC_REG_STOPLL_C2:
        {
          int16_t hC1;
          int16_t hC2;
          STO_PLL_GetObserverGains(&STO_PLL_M1, &hC1, &hC2);
          STO_PLL_SetObserverGains(&STO_PLL_M1, hC1, (int16_t)regdata16);
          break;
        }

        case MC_REG_STOPLL_KI:
        {
          PID_SetKI (&(&STO_PLL_M1)->PIRegulator, (int16_t)regdata16);
          break;
        }

        case MC_REG_STOPLL_KP:
        {
          PID_SetKP (&(&STO_PLL_M1)->PIRegulator, (int16_t)regdata16);
          break;
        }

        case MC_REG_STOPLL_EL_ANGLE:
        case MC_REG_STOPLL_ROT_SPEED:
        case MC_REG_STOPLL_I_ALPHA:
        case MC_REG_STOPLL_I_BETA:
        case MC_REG_STOPLL_BEMF_ALPHA:
        case MC_REG_STOPLL_BEMF_BETA:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }

        case MC_REG_DAC_USER1:
        case MC_REG_DAC_USER2:
          break;

        case MC_REG_FF_VQ:
        case MC_REG_FF_VD:
        case MC_REG_FF_VQ_PIOUT:
        case MC_REG_FF_VD_PIOUT:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }

        case MC_REG_OVERVOLTAGETHRESHOLD:
        {
          SCC_SetOverVoltageThreshold(&SCC,regdata16);
          break;
        }

        case MC_REG_UNDERVOLTAGETHRESHOLD:
        {
          SCC_SetUnderVoltageThreshold(&SCC,regdata16);
          break;
        }

        case MC_REG_SC_PWM_FREQUENCY:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }

        case MC_REG_SPEED_KP_DIV:
        {
          PID_SetKPDivisorPOW2(&PIDSpeedHandle_M1, regdata16);
          break;
        }

        case MC_REG_SPEED_KI_DIV:
        {
          PID_SetKIDivisorPOW2(&PIDSpeedHandle_M1, regdata16);
          break;
        }

        case MC_REG_SPEED_KD_DIV:
        {
          PID_SetKDDivisorPOW2(&PIDSpeedHandle_M1, regdata16);
          break;
        }

        case MC_REG_I_D_KP_DIV:
        {
          PID_SetKPDivisorPOW2(&PIDIdHandle_M1, regdata16);
          break;
        }

        case MC_REG_I_D_KI_DIV:
        {
          PID_SetKIDivisorPOW2(&PIDIdHandle_M1, regdata16);
          break;
        }

        case MC_REG_I_D_KD_DIV:
        {
          PID_SetKDDivisorPOW2(&PIDIdHandle_M1, regdata16);
          break;
        }

        case MC_REG_I_Q_KP_DIV:
        {
          PID_SetKPDivisorPOW2(&PIDIqHandle_M1, regdata16);
          break;
        }

        case MC_REG_I_Q_KI_DIV:
        {
          PID_SetKIDivisorPOW2(&PIDIqHandle_M1, regdata16);
          break;
        }

        case MC_REG_I_Q_KD_DIV:
        {
          PID_SetKDDivisorPOW2(&PIDIqHandle_M1, regdata16);
          break;
        }

        case MC_REG_STOPLL_KI_DIV:
        {
          PID_SetKIDivisorPOW2 (&(&STO_PLL_M1)->PIRegulator,regdata16);
          break;
        }

        case MC_REG_STOPLL_KP_DIV:
        {
          PID_SetKPDivisorPOW2 (&(&STO_PLL_M1)->PIRegulator,regdata16);
          break;
        }

        case MC_REG_FLUXWK_KP_DIV:
        {
          PID_SetKPDivisorPOW2 (&PIDFluxWeakeningHandle_M1,regdata16);
          break;
        }

        case MC_REG_FLUXWK_KI_DIV:
        {
          PID_SetKIDivisorPOW2 (&PIDFluxWeakeningHandle_M1,regdata16);
          break;
        }

        case MC_REG_PULSE_VALUE:
        case MC_REG_BEMF_ZCR:
        case MC_REG_BEMF_U:
        case MC_REG_BEMF_V:
        case MC_REG_BEMF_W:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }

        default:
        {
          retVal = MCP_ERROR_UNKNOWN_REG;
          break;
        }
      }
      *size = 2;
      break;
    }

    case TYPE_DATA_32BIT:
    {
      uint32_t regdata32 = *(uint32_t *)data; //cstat !MISRAC2012-Rule-11.3

      switch (regID)
      {
        case MC_REG_FAULTS_FLAGS:
        case MC_REG_SPEED_MEAS:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }

        case MC_REG_SPEED_REF:
        {
          MCI_ExecSpeedRamp(pMCIN,((((int16_t)regdata32) * ((int16_t)SPEED_UNIT)) / (int16_t)U_RPM), 0);
          break;
        }

        case MC_REG_STOPLL_EST_BEMF:
        case MC_REG_STOPLL_OBS_BEMF:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }

        case MC_REG_FF_1Q:
        {
          FF_M1.wConstant_1Q = (int32_t)regdata32;
          break;
        }

        case MC_REG_FF_1D:
        {
          FF_M1.wConstant_1D = (int32_t)regdata32;
          break;
        }

        case MC_REG_FF_2:
        {
          FF_M1.wConstant_2 = (int32_t)regdata32;
          break;
        }

        case MC_REG_SC_RS:
        case MC_REG_SC_LS:
        case MC_REG_SC_KE:
        case MC_REG_SC_VBUS:
        case MC_REG_SC_MEAS_NOMINALSPEED:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }

        case MC_REG_SC_CURRENT:
        { /* Profiler is supported only by series supporting unaligned access */
          if(SCC.sm_state==SCC_IDLE || SCC_CALIBRATION_END==SCC.sm_state)
          {
            float fregdata = *(float*)data; //cstat !MISRAC2012-Rule-11.3
            SCC_SetNominalCurrent(&SCC, fregdata);
          }
          else{
              retVal = MCP_ERROR_RO_REG;
          }
          break;
        }

        case MC_REG_SC_SPDBANDWIDTH:
        {
          float fregdata = *(float*)data; //cstat !MISRAC2012-Rule-11.3
          OTT_SetSpeedRegulatorBandwidth(&OTT, fregdata);
          break;
        }

        case MC_REG_SC_LDLQRATIO:
        {
          float fregdata = *(float*)data; //cstat !MISRAC2012-Rule-11.3
          SCC_SetLdLqRatio(&SCC, fregdata);
          break;
        }

        case MC_REG_SC_NOMINAL_SPEED:
        {
          SCC_SetNominalSpeed (&SCC, (int32_t) regdata32);
          break;
        }

        case MC_REG_SC_CURRBANDWIDTH:
        {
          float fregdata = *(float*)data; //cstat !MISRAC2012-Rule-11.3
          SCC_SetCurrentBandwidth(&SCC, fregdata);
          break;
        }
        case MC_REG_RESISTOR_OFFSET:
        {
          float fregdata = *(float*)data;
          SCC_SetResistorOffset(&SCC,fregdata);
          break;
        }
        case MC_REG_SC_J:
        case MC_REG_SC_F:
        case MC_REG_SC_MAX_CURRENT:
        case MC_REG_SC_STARTUP_SPEED:
        case MC_REG_SC_STARTUP_ACC:
        {
          retVal = MCP_ERROR_RO_REG;
          break;
        }

        default:
        {
          retVal = MCP_ERROR_UNKNOWN_REG;
          break;
        }
      }
      *size = 4;
      break;
    }

    case TYPE_DATA_STRING:
    {
      const char_t *charData = (const char_t *)data;
      char_t *dummy = (char_t *)data;
      retVal = MCP_ERROR_RO_REG;
      /* Used to compute String length stored in RXBUFF even if Reg does not exist */
      /* It allows to jump to the next command in the buffer */
      (void)RI_MovString(charData, dummy, size, dataAvailable);
      break;
    }

    case TYPE_DATA_RAW:
    {
      uint16_t rawSize = *(uint16_t *)data; //cstat !MISRAC2012-Rule-11.3
      /* The size consumed by the structure is the structure size + 2 bytes used to store the size */
      *size = rawSize + 2U;
      uint8_t *rawData = data; /* rawData points to the first data (after size extraction) */
      rawData++;
      rawData++;

      if (*size > (uint16_t)dataAvailable)
      {
        /* The decoded size of the raw structure can not match with transmitted buffer, error in buffer
           construction */
        *size = 0;
        retVal = MCP_ERROR_BAD_RAW_FORMAT; /* This error stop the parsing of the CMD buffer */
      }
      else
      {
        switch (regID)
        {
          case MC_REG_APPLICATION_CONFIG:
          case MC_REG_MOTOR_CONFIG:
          case MC_REG_GLOBAL_CONFIG:
          case MC_REG_FOCFW_CONFIG:
          {
            retVal = MCP_ERROR_RO_REG;
            break;
          }

          case MC_REG_SPEED_RAMP:
          {
            int32_t rpm;
            uint16_t duration;

            rpm = *(int32_t *)rawData; //cstat !MISRAC2012-Rule-11.3
            duration = *(uint16_t *)&rawData[4]; //cstat !MISRAC2012-Rule-11.3
            MCI_ExecSpeedRamp(pMCIN, (int16_t)((rpm * SPEED_UNIT) / U_RPM), duration);
            break;
          }

          case MC_REG_TORQUE_RAMP:
          {
            uint32_t torque;
            uint16_t duration;

            torque = *(uint32_t *)rawData; //cstat !MISRAC2012-Rule-11.3
            duration = *(uint16_t *)&rawData[4]; //cstat !MISRAC2012-Rule-11.3
            MCI_ExecTorqueRamp(pMCIN, (int16_t)torque, duration);
            break;
          }

          case MC_REG_REVUP_DATA:
          {
            int32_t rpm;
            RevUpCtrl_PhaseParams_t revUpPhase;
            uint8_t i;
            uint8_t nbrOfPhase = (((uint8_t)rawSize) / 8U);

            if (((0U != ((rawSize) % 8U))) || ((nbrOfPhase > RUC_MAX_PHASE_NUMBER) != 0))
            {
              retVal = MCP_ERROR_BAD_RAW_FORMAT;
            }
            else
            {
              for (i = 0; i <nbrOfPhase; i++)
              {
              rpm = *(int32_t *) &rawData[i * 8U]; //cstat !MISRAC2012-Rule-11.3
              revUpPhase.hFinalMecSpeedUnit = (((int16_t)rpm) * ((int16_t)SPEED_UNIT)) / ((int16_t)U_RPM);
              revUpPhase.hFinalTorque = *((int16_t *) &rawData[4U + (i * 8U)]); //cstat !MISRAC2012-Rule-11.3
              revUpPhase.hDurationms  = *((uint16_t *) &rawData[6U +(i * 8U)]); //cstat !MISRAC2012-Rule-11.3
              (void)RUC_SetPhase(&RevUpControlM1, i, &revUpPhase);
              }
            }
            break;
          }

          case MC_REG_CURRENT_REF:
          {
            qd_t currComp;
            currComp.q = *((int16_t *) rawData); //cstat !MISRAC2012-Rule-11.3
            currComp.d = *((int16_t *) &rawData[2]); //cstat !MISRAC2012-Rule-11.3
            MCI_SetCurrentReferences(pMCIN, currComp);
            break;
          }

          case MC_REG_ASYNC_UARTA:
          {
            retVal =  MCPA_cfgLog (&MCPA_UART_A, rawData);
            break;
          }

          default:
          {
            retVal = MCP_ERROR_UNKNOWN_REG;
            break;
          }
        }
      }
      break;
    }

    default:
    {
      retVal = MCP_ERROR_BAD_DATA_TYPE;
      *size =0; /* From this point we are not able anymore to decode the RX buffer */
      break;
    }
  }
  return (retVal);
}

uint8_t RI_GetRegisterGlobal(uint16_t regID,uint8_t typeID,uint8_t * data,uint16_t *size,int16_t freeSpace){
    uint8_t retVal = MCP_CMD_OK;
    uint8_t motorID=0;
    MCI_Handle_t *pMCIN = &Mci[motorID];
    switch (typeID)
    {
      case TYPE_DATA_8BIT:
      {
        if (freeSpace > 0)
        {
          switch (regID)
          {
            default:
            {
              retVal = MCP_ERROR_UNKNOWN_REG;
              break;
            }
          }
          *size = 1;
        }
        else
        {
          retVal = MCP_ERROR_NO_TXSYNC_SPACE;
        }
        break;
      }

      case TYPE_DATA_16BIT:
      {
       int16_t *regdata16 = (int16_t *) data; //cstat !MISRAC2012-Rule-11.3
        if (freeSpace >= 2)
        {
          switch (regID)
          {
            case MC_REG_DAC_OUT1:
            {
              *regdata16 = (int16_t)DAC_GetChannelConfig(&DAC_Handle , DAC_CH1);
              break;
            }

            case MC_REG_DAC_OUT2:
            {
              *regdata16 = (int16_t)DAC_GetChannelConfig(&DAC_Handle , DAC_CH2);
              break;
            }
            case MC_REG_DAC_USER1:
            case MC_REG_DAC_USER2:
              break;

            default:
            {
              retVal = MCP_ERROR_UNKNOWN_REG;
              break;
            }
          }
          *size = 2;
        }
        else
        {
          retVal = MCP_ERROR_NO_TXSYNC_SPACE;
        }
        break;
      }

      case TYPE_DATA_32BIT:
      {
        uint32_t *regdataU32 = (uint32_t *)data; //cstat !MISRAC2012-Rule-11.3
        if (freeSpace >= 4)
        {
          switch (regID)
          {
            case MC_REG_PERF_CPU_LOAD:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = MC_Perf_GetCPU_Load(pMCIN->pPerfMeasure);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }

            case MC_REG_PERF_MIN_CPU_LOAD:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = MC_Perf_GetMinCPU_Load(pMCIN->pPerfMeasure);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }

            case MC_REG_PERF_MAX_CPU_LOAD:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = MC_Perf_GetMaxCPU_Load(pMCIN->pPerfMeasure);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }

            default:
            {
              retVal = MCP_ERROR_UNKNOWN_REG;
              break;
            }
          }
          *size = 4;
        }
        else
        {
          retVal = MCP_ERROR_NO_TXSYNC_SPACE;
        }
        break;
      }

      case TYPE_DATA_STRING:
      {
        char_t *charData = (char_t *)data;
        switch (regID)
        {
          case MC_REG_FW_NAME:
            retVal = RI_MovString (FIRMWARE_NAME ,charData, size, freeSpace);
            break;

          case MC_REG_CTRL_STAGE_NAME:
          {
            retVal = RI_MovString (CTL_BOARD ,charData, size, freeSpace);
            break;
          }
          default:
          {

            retVal = MCP_ERROR_UNKNOWN_REG;
            *size= 0 ; /* */

            break;
          }
        }
        break;

      }
      case TYPE_DATA_RAW:
      {
        /* First 2 bytes of the answer is reserved to the size */
        uint16_t *rawSize = (uint16_t *)data; //cstat !MISRAC2012-Rule-11.3
        uint8_t * rawData = data;
        rawData++;
        rawData++;

        switch (regID)
        {
          case MC_REG_GLOBAL_CONFIG:
          {
            *rawSize = (uint16_t)sizeof(GlobalConfig_reg_t);
            if (((*rawSize) + 2U) > (uint16_t)freeSpace)
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else
            {
              (void)memcpy(rawData, &globalConfig_reg, sizeof(GlobalConfig_reg_t));
            }
            break;
          }
          case MC_REG_ASYNC_UARTA:
          case MC_REG_ASYNC_UARTB:
          case MC_REG_ASYNC_STLNK:
          default:
          {
            retVal = MCP_ERROR_UNKNOWN_REG;
            break;
          }
        }

        /* Size of the answer is size of the data + 2 bytes containing data size */
        *size = (*rawSize) + 2U;
        break;
      }

      default:
      {
        retVal = MCP_ERROR_BAD_DATA_TYPE;
        break;
      }
    }
  return (retVal);
}

  uint8_t RI_GetRegisterMotor1(uint16_t regID,uint8_t typeID,uint8_t * data,uint16_t *size,int16_t freeSpace) {
    uint8_t retVal = MCP_CMD_OK;
    uint8_t motorID=0;
    MCI_Handle_t *pMCIN = &Mci[motorID];
    BusVoltageSensor_Handle_t* BusVoltageSensor= &BusVoltageSensor_M1._Super;
    switch (typeID)
    {
      case TYPE_DATA_8BIT:
      {
        if (freeSpace > 0)
        {
          switch (regID)
          {
            case MC_REG_STATUS:
            {
              *data = (uint8_t)MCI_GetSTMState(pMCIN);
              break;
            }

            case MC_REG_CONTROL_MODE:
            {
              *data = (uint8_t)MCI_GetControlMode(pMCIN);
              break;
            }

            case MC_REG_RUC_STAGE_NBR:
            {
              *data = (uint8_t)RUC_GetNumberOfPhases(&RevUpControlM1);
              break;
            }

            case MC_REG_SC_CHECK:
            {
              *data = (uint8_t) 1u;
              break;
            }

            case MC_REG_SC_STATE:
            {
              uint8_t state ;
              state = SCC_GetState(&SCC);
              state += OTT_GetState (&OTT);
              *data = state;
              break;
            }

            case MC_REG_SC_STEPS:
            {
              uint8_t steps ;
              steps = SCC_GetSteps(&SCC);
              steps += OTT_GetSteps (&OTT);
              *data = steps-1u;
              break;
            }

            case MC_REG_SC_PP:
            {
              *data = SPD_GetElToMecRatio(&STO_PLL_M1._Super);
              break;
            }

            case MC_REG_SC_FOC_REP_RATE:
            {
              *data = SCC_GetFOCRepRate(&SCC);
              break;
            }

            case MC_REG_SC_COMPLETED:
            {
              *data = OTT_IsMotorAlreadyProfiled(&OTT);
              break;
             }

            case MC_REG_HT_STATE:
            {
              *data = (uint8_t) HT.sm_state;
              break;
            }

            case MC_REG_HT_PROGRESS:
            {
              *data = HT.bProgressPercentage;
              break;
            }

            case MC_REG_HT_PLACEMENT:
            {
              *data = HT.bPlacement;
              break;
            }

            default:
            {
              retVal = MCP_ERROR_UNKNOWN_REG;
              break;
            }
          }
          *size = 1;
        }
        else
        {
          retVal = MCP_ERROR_NO_TXSYNC_SPACE;
        }
        break;
      }

      case TYPE_DATA_16BIT:
      {
        uint16_t *regdataU16 = (uint16_t *)data; //cstat !MISRAC2012-Rule-11.3
        int16_t *regdata16 = (int16_t *) data; //cstat !MISRAC2012-Rule-11.3

        if (freeSpace >= 2)
        {
          switch (regID)
          {
            case MC_REG_SPEED_KP:
            {
              *regdata16 = PID_GetKP(&PIDSpeedHandle_M1);
              break;
            }

            case MC_REG_SPEED_KI:
            {
              *regdata16 = PID_GetKI(&PIDSpeedHandle_M1);
              break;
            }

            case MC_REG_SPEED_KD:
            {
              *regdata16 = PID_GetKD(&PIDSpeedHandle_M1);
              break;
            }

            case MC_REG_I_Q_KP:
            {
              *regdata16 = PID_GetKP(&PIDIqHandle_M1);
              break;
            }

            case MC_REG_I_Q_KI:
            {
              *regdata16 = PID_GetKI(&PIDIqHandle_M1);
              break;
            }

            case MC_REG_I_Q_KD:
            {
              *regdata16 = PID_GetKD(&PIDIqHandle_M1);
              break;
            }

            case MC_REG_I_D_KP:
            {
              *regdata16 = PID_GetKP(&PIDIdHandle_M1);
              break;
            }

            case MC_REG_I_D_KI:
            {
              *regdata16 = PID_GetKI(&PIDIdHandle_M1);
              break;
            }

            case MC_REG_I_D_KD:
            {
              *regdata16 = PID_GetKD(&PIDIdHandle_M1);
              break;
            }

            case MC_REG_FLUXWK_KP:
            {
              *regdata16 = PID_GetKP(&PIDFluxWeakeningHandle_M1);
              break;
            }

            case MC_REG_FLUXWK_KI:
            {
              *regdata16 = PID_GetKI(&PIDFluxWeakeningHandle_M1);
              break;
            }

            case MC_REG_FLUXWK_BUS:
            {
              *regdataU16 = FW_GetVref(&FW_M1);
              break;
            }

            case MC_REG_FLUXWK_BUS_MEAS:
            {
              *regdata16 = (int16_t)FW_GetAvVPercentage(&FW_M1);
              break;
            }

            case MC_REG_BUS_VOLTAGE:
            {
              *regdataU16 = VBS_GetAvBusVoltage_V(BusVoltageSensor);
              break;
            }

            case MC_REG_HEATS_TEMP:
            {
              *regdata16 = NTC_GetAvTemp_C(&TempSensor_M1);
              break;
            }

            case MC_REG_I_A:
            {
              *regdata16 = MCI_GetIab(pMCIN).a;
              break;
            }

            case MC_REG_I_B:
            {
              *regdata16 = MCI_GetIab(pMCIN).b;
              break;
            }

            case MC_REG_I_ALPHA_MEAS:
            {
              *regdata16 = MCI_GetIalphabeta(pMCIN).alpha;
              break;
            }

            case MC_REG_I_BETA_MEAS:
            {
              *regdata16 = MCI_GetIalphabeta(pMCIN).beta;
              break;
            }

            case MC_REG_I_Q_MEAS:
            {
              *regdata16 = MCI_GetIqd(pMCIN).q;
              break;
            }

            case MC_REG_I_D_MEAS:
            {
              *regdata16 = MCI_GetIqd(pMCIN).d;
              break;
            }

            case MC_REG_I_Q_REF:
            {
              *regdata16 = MCI_GetIqdref(pMCIN).q;
              break;
            }

            case MC_REG_I_D_REF:
            {
              *regdata16 = MCI_GetIqdref(pMCIN).d;
              break;
            }

            case MC_REG_V_Q:
            {
              *regdata16 = MCI_GetVqd(pMCIN).q;
              break;
            }

            case MC_REG_V_D:
            {
              *regdata16 = MCI_GetVqd(pMCIN).d;
              break;
            }

            case MC_REG_V_ALPHA:
            {
              *regdata16 = MCI_GetValphabeta(pMCIN).alpha;
              break;
            }

            case MC_REG_V_BETA:
            {
              *regdata16 = MCI_GetValphabeta(pMCIN).beta;
              break;
            }

            case MC_REG_HALL_EL_ANGLE:
            {
              //cstat !MISRAC2012-Rule-11.3
              *regdata16 = SPD_GetElAngle ((SpeednPosFdbk_Handle_t*) &HALL_M1);
              break;
            }

            case MC_REG_HALL_SPEED:
            {
              //cstat !MISRAC2012-Rule-11.3
              *regdata16 = SPD_GetS16Speed ((SpeednPosFdbk_Handle_t*) &HALL_M1);
              break;
            }

            case MC_REG_STOPLL_EL_ANGLE:
            {
              //cstat !MISRAC2012-Rule-11.3
              *regdata16 = SPD_GetElAngle((SpeednPosFdbk_Handle_t *)&STO_PLL_M1);
              break;
            }

            case MC_REG_STOPLL_ROT_SPEED:
            {
              //cstat !MISRAC2012-Rule-11.3
              *regdata16 = SPD_GetS16Speed((SpeednPosFdbk_Handle_t *)&STO_PLL_M1);
              break;
            }

            case MC_REG_STOPLL_I_ALPHA:
            {
              *regdata16 = STO_PLL_GetEstimatedCurrent(&STO_PLL_M1).alpha;
              break;
            }

            case MC_REG_STOPLL_I_BETA:
            {
              *regdata16 = STO_PLL_GetEstimatedCurrent(&STO_PLL_M1).beta;
              break;
            }

            case MC_REG_STOPLL_BEMF_ALPHA:
            {
              *regdata16 = STO_PLL_GetEstimatedBemf(&STO_PLL_M1).alpha;
              break;
            }

            case MC_REG_STOPLL_BEMF_BETA:
            {
              *regdata16 = STO_PLL_GetEstimatedBemf(&STO_PLL_M1).beta;
              break;
            }

            case MC_REG_STOPLL_C1:
            {
              int16_t hC1;
              int16_t hC2;
              STO_PLL_GetObserverGains(&STO_PLL_M1, &hC1, &hC2);
              *regdata16 = hC1;
              break;
            }

            case MC_REG_STOPLL_C2:
            {
              int16_t hC1;
              int16_t hC2;
              STO_PLL_GetObserverGains(&STO_PLL_M1, &hC1, &hC2);
              *regdata16 = hC2;
              break;
            }

            case MC_REG_STOPLL_KI:
            {
              *regdata16 = PID_GetKI (&(&STO_PLL_M1)->PIRegulator);
              break;
            }

            case MC_REG_STOPLL_KP:
            {
              *regdata16 = PID_GetKP (&(&STO_PLL_M1)->PIRegulator);
              break;
            }

            case MC_REG_DAC_USER1:
            case MC_REG_DAC_USER2:
              break;

            case MC_REG_FF_VQ:
            {
              *regdata16 = FF_GetVqdff(&FF_M1).q;
              break;
            }

            case MC_REG_FF_VD:
            {
              *regdata16 = FF_GetVqdff(&FF_M1).d;
              break;
            }

            case MC_REG_FF_VQ_PIOUT:
            {
              *regdata16 = FF_GetVqdAvPIout(&FF_M1).q;
              break;
            }

            case MC_REG_FF_VD_PIOUT:
            {
              *regdata16 = FF_GetVqdAvPIout(&FF_M1).d;
              break;
            }

            case MC_REG_SC_PWM_FREQUENCY:
              *regdataU16 = SCC_GetPWMFrequencyHz(&SCC);
              break;
            case MC_REG_OVERVOLTAGETHRESHOLD:
            {
              *regdataU16 = SCC_GetOverVoltageThreshold(&SCC);
              break;
            }

            case MC_REG_UNDERVOLTAGETHRESHOLD:
              {
                *regdata16 = SCC_GetUnderVoltageThreshold(&SCC);
              break;
            }

            case MC_REG_SPEED_KP_DIV:
            {
              *regdataU16 = (uint16_t)PID_GetKPDivisorPOW2(&PIDSpeedHandle_M1);
              break;
            }

            case MC_REG_SPEED_KI_DIV:
            {
              *regdataU16 = (uint16_t)PID_GetKIDivisorPOW2(&PIDSpeedHandle_M1);
              break;
            }

            case MC_REG_SPEED_KD_DIV:
            {
              *regdataU16 = PID_GetKDDivisorPOW2(&PIDSpeedHandle_M1);
              break;
            }
            case MC_REG_I_D_KP_DIV:
            {
              *regdataU16 = PID_GetKPDivisorPOW2(&PIDIdHandle_M1);
              break;
            }

            case MC_REG_I_D_KI_DIV:
            {
              *regdataU16 = PID_GetKIDivisorPOW2(&PIDIdHandle_M1);
              break;
            }

            case MC_REG_I_D_KD_DIV:
            {
              *regdataU16 = PID_GetKDDivisorPOW2(&PIDIdHandle_M1);
              break;
            }

            case MC_REG_I_Q_KP_DIV:
            {
              *regdataU16 = PID_GetKPDivisorPOW2(&PIDIqHandle_M1);
              break;
            }

            case MC_REG_I_Q_KI_DIV:
            {
              *regdataU16 = PID_GetKIDivisorPOW2(&PIDIqHandle_M1);
              break;
            }

            case MC_REG_I_Q_KD_DIV:
            {
              *regdataU16 = PID_GetKDDivisorPOW2(&PIDIqHandle_M1);
              break;
            }

            case MC_REG_STOPLL_KI_DIV:
            {
              *regdataU16 = PID_GetKIDivisorPOW2(&(&STO_PLL_M1)->PIRegulator);
              break;
            }

            case MC_REG_STOPLL_KP_DIV:
            {
              *regdataU16 = PID_GetKPDivisorPOW2(&(&STO_PLL_M1)->PIRegulator);
              break;
            }

            case MC_REG_FLUXWK_KP_DIV:
            {
              *regdataU16 = PID_GetKPDivisorPOW2(&PIDFluxWeakeningHandle_M1);
              break;
            }

            case MC_REG_FLUXWK_KI_DIV:
            {
              *regdataU16 = PID_GetKIDivisorPOW2(&PIDFluxWeakeningHandle_M1);
              break;
            }

            default:
            {
              retVal = MCP_ERROR_UNKNOWN_REG;
              break;
            }
          }
          *size = 2;
        }
        else
        {
          retVal = MCP_ERROR_NO_TXSYNC_SPACE;
        }
        break;
      }

      case TYPE_DATA_32BIT:
      {
        uint32_t *regdataU32 = (uint32_t *)data; //cstat !MISRAC2012-Rule-11.3
        int32_t *regdata32 = (int32_t *)data; //cstat !MISRAC2012-Rule-11.3

        if (freeSpace >= 4)
        {
          switch (regID)
          {
            case MC_REG_FAULTS_FLAGS:
            {
              *regdataU32 = MCI_GetFaultState(pMCIN);
              break;
            }
            case MC_REG_SPEED_MEAS:
            {
              *regdata32 = (((int32_t)MCI_GetAvrgMecSpeedUnit(pMCIN) * U_RPM) / SPEED_UNIT);
              break;
            }

            case MC_REG_SPEED_REF:
            {
              *regdata32 = (((int32_t)MCI_GetMecSpeedRefUnit(pMCIN) * U_RPM) / SPEED_UNIT);
              break;
            }

            case MC_REG_STOPLL_EST_BEMF:
            {
              *regdata32 = STO_PLL_GetEstimatedBemfLevel(&STO_PLL_M1);
              break;
            }

            case MC_REG_STOPLL_OBS_BEMF:
            {
              *regdata32 = STO_PLL_GetObservedBemfLevel(&STO_PLL_M1);
              break;
            }

            case MC_REG_FF_1Q:
            {
              *regdata32 = FF_M1.wConstant_1Q;
              break;
            }

            case MC_REG_FF_1D:
            {
              *regdata32 = FF_M1.wConstant_1D;
              break;
            }

            case MC_REG_FF_2:
            {
              *regdata32 = FF_M1.wConstant_2;
              break;
            }

            case MC_REG_MOTOR_POWER:
            {
              FloatToU32 ReadVal; //cstat !MISRAC2012-Rule-19.2
              ReadVal.Float_Val = PQD_GetAvrgElMotorPowerW(pMPM[M1]);
              *regdataU32 = ReadVal.U32_Val; //cstat !UNION-type-punning
              break;
            }

            case MC_REG_SC_RS:
            {
              *regdataU32 = SCC_GetRs(&SCC);
              break;
            }

            case MC_REG_SC_LS:
            {
              *regdataU32 = SCC_GetLs(&SCC);
              break;
            }

            case MC_REG_SC_KE:
            {
              *regdataU32 = SCC_GetKe(&SCC);
              break;
            }

            case MC_REG_SC_VBUS:
            {
              *regdataU32 = SCC_GetVbus(&SCC);
              break;
            }

            case MC_REG_SC_MEAS_NOMINALSPEED:
            {
              *regdataU32 = OTT_GetNominalSpeed(&OTT);
              break;
            }

            case MC_REG_SC_CURRENT:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = SCC_GetNominalCurrent(&SCC);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }

            case MC_REG_SC_SPDBANDWIDTH:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = OTT_GetSpeedRegulatorBandwidth(&OTT);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }

            case MC_REG_SC_LDLQRATIO:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = SCC_GetLdLqRatio(&SCC);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }

            case MC_REG_SC_NOMINAL_SPEED:
            {
              *regdata32 = SCC_GetNominalSpeed(&SCC);
              break;
            }

            case MC_REG_SC_CURRBANDWIDTH:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = SCC_GetCurrentBandwidth(&SCC);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }

            case MC_REG_SC_J:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = OTT_GetJ(&OTT);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }

            case MC_REG_SC_F:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = OTT_GetF(&OTT);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }

            case MC_REG_SC_MAX_CURRENT:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val = SCC_GetStartupCurrentAmp(&SCC);
              *regdataU32 = ReadVal.U32_Val;
              break;
            }

            case MC_REG_SC_STARTUP_SPEED:
            {
              *regdata32 = SCC_GetEstMaxOLSpeed(&SCC);
              break;
            }

            case MC_REG_SC_STARTUP_ACC:
            {
              *regdata32 = SCC_GetEstMaxAcceleration(&SCC);
              break;
            }
            case MC_REG_RESISTOR_OFFSET:
            {
              FloatToU32 ReadVal;
              ReadVal.Float_Val=SCC_GetResistorOffset(&SCC);
              *regdata32 =ReadVal.U32_Val;
              break;
            }

            default:
            {
              retVal = MCP_ERROR_UNKNOWN_REG;
              break;
            }
          }
          *size = 4;
        }
        else
        {
          retVal = MCP_ERROR_NO_TXSYNC_SPACE;
        }
        break;
      }

      case TYPE_DATA_STRING:
      {
        char_t *charData = (char_t *)data;
        switch (regID)
        {
          case MC_REG_PWR_STAGE_NAME:
          {
            retVal = RI_MovString (PWR_BOARD_NAME[motorID], charData, size, freeSpace);
            break;
          }

          case MC_REG_MOTOR_NAME:
          {
            retVal = RI_MovString (MotorConfig_reg[motorID]->name ,charData, size, freeSpace);
            break;
          }

          default:
          {
            retVal = MCP_ERROR_UNKNOWN_REG;
            *size= 0 ; /* */
            break;
          }
        }
        break;
      }

      case TYPE_DATA_RAW:
      {
        /* First 2 bytes of the answer is reserved to the size */
        uint16_t *rawSize = (uint16_t *)data; //cstat !MISRAC2012-Rule-11.3
        uint8_t * rawData = data;
        rawData++;
        rawData++;

        switch (regID)
        {
          case MC_REG_APPLICATION_CONFIG:
          {
            *rawSize = (uint16_t)sizeof(ApplicationConfig_reg_t);
            if (((*rawSize) + 2U) > (uint16_t)freeSpace)
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else
            {
              ApplicationConfig_reg_t const *pApplicationConfig_reg = ApplicationConfig_reg[motorID];
              (void)memcpy(rawData, (const uint8_t *)pApplicationConfig_reg, sizeof(ApplicationConfig_reg_t));
            }
            break;
          }

          case MC_REG_MOTOR_CONFIG:
          {
            *rawSize = (uint16_t)sizeof(MotorConfig_reg_t);
            if (((*rawSize) + 2U) > (uint16_t)freeSpace)
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else
            {
              MotorConfig_reg_t const *pMotorConfig_reg = MotorConfig_reg[motorID];
              (void)memcpy(rawData, (const uint8_t *)pMotorConfig_reg, sizeof(MotorConfig_reg_t));
            }
            break;
          }

          case MC_REG_FOCFW_CONFIG:
          {
            *rawSize = (uint16_t)sizeof(FOCFwConfig_reg_t);
            if (((*rawSize) + 2U) > (uint16_t)freeSpace)
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else
            {
              FOCFwConfig_reg_t const *pFOCConfig_reg = FOCConfig_reg[motorID];
              (void)memcpy(rawData, (const uint8_t *)pFOCConfig_reg, sizeof(FOCFwConfig_reg_t));
            }

            break;
          }
          case MC_REG_SCALE_CONFIG:
          {
            *rawSize = 12;
            if ((*rawSize) +2U > (uint16_t)freeSpace)
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else
            {
              memcpy(rawData, &scaleParams_M1, sizeof(ScaleParams_t) );
            }
            break;
          }
          case MC_REG_SPEED_RAMP:
          {
            int32_t *rpm = (int32_t *)rawData; //cstat !MISRAC2012-Rule-11.3
            uint16_t *duration = (uint16_t *)&rawData[4]; //cstat !MISRAC2012-Rule-11.3
            *rpm = (((int32_t)MCI_GetLastRampFinalSpeed(pMCIN) * U_RPM) / (int32_t)SPEED_UNIT);
            *duration = MCI_GetLastRampFinalDuration(pMCIN);
            *rawSize = 6;
            break;
          }

          case MC_REG_TORQUE_RAMP:
          {
            int16_t *torque = (int16_t *)rawData; //cstat !MISRAC2012-Rule-11.3
            uint16_t *duration = (uint16_t *)&rawData[2]; //cstat !MISRAC2012-Rule-11.3

            *rawSize = 4;
            *torque = MCI_GetLastRampFinalTorque(pMCIN);
            *duration = MCI_GetLastRampFinalDuration(pMCIN) ;
            break;
          }

          case MC_REG_REVUP_DATA:
          {
            int32_t *rpm;
            uint16_t *finalTorque;
            uint16_t *durationms;
            RevUpCtrl_PhaseParams_t revUpPhase;
            uint8_t i;

            *rawSize = (uint16_t)RUC_MAX_PHASE_NUMBER*8U;
            if (((*rawSize) + 2U) > (uint16_t)freeSpace)
            {
              retVal = MCP_ERROR_NO_TXSYNC_SPACE;
            }
            else
            {
              for (i = 0; i <RUC_MAX_PHASE_NUMBER; i++)
              {
                (void)RUC_GetPhase( &RevUpControlM1 ,i, &revUpPhase);
                rpm = (int32_t *)&data[2U + (i * 8U)];  //cstat !MISRAC2012-Rule-11.3
                *rpm = (((int32_t)revUpPhase.hFinalMecSpeedUnit) * U_RPM) / SPEED_UNIT; //cstat !MISRAC2012-Rule-11.3
                finalTorque = (uint16_t *)&data[6U + (i * 8U)]; //cstat !MISRAC2012-Rule-11.3
                *finalTorque = (uint16_t)revUpPhase.hFinalTorque; //cstat !MISRAC2012-Rule-11.3
                durationms  = (uint16_t *)&data[8U + (i * 8U)]; //cstat !MISRAC2012-Rule-11.3
                *durationms  = revUpPhase.hDurationms;
              }
            }
            break;
          }

          case MC_REG_CURRENT_REF:
          {
            uint16_t *iqref = (uint16_t *)rawData; //cstat !MISRAC2012-Rule-11.3
            uint16_t *idref = (uint16_t *)&rawData[2]; //cstat !MISRAC2012-Rule-11.3

            *rawSize = 4;
            *iqref = (uint16_t)MCI_GetIqdref(pMCIN).q;
            *idref = (uint16_t)MCI_GetIqdref(pMCIN).d;
            break;
          }

          case MC_REG_HT_HEW_PINS:
            {
              *rawSize = 4;
              *rawData = HT.bNewH1;
              *(rawData+1) = HT.bNewH2;
              *(rawData+2) = HT.bNewH3;
              *(rawData+3) = (uint8_t) 0U; /* Padding Raw structure must be 16 bits aligned */
              break;
            }

          case MC_REG_HT_CONNECTED_PINS:
            {
              *rawSize = 4;
              *rawData = HT.H1Connected;
              *(rawData+1) = HT.H2Connected;
              *(rawData+2) = HT.H3Connected;
              *(rawData+3) = (uint8_t) 0U; /* Padding Raw structure must be 16 bits aligned */
              break;
            }

          case MC_REG_HT_PHASE_SHIFT:
            {
              int16_t *rawData16 = (int16_t *) rawData;
              *rawSize = 28; /*14 16 bits values */
              *rawData16 = HT.hPhaseShiftCircularMean;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean5_1;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean1_3;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean3_2;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean2_6;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean6_4;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean4_5;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMeanNeg;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean5_4;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean4_6;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean6_2;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean2_3;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean3_1;
              rawData16++;
              *rawData16 = HT.hPhaseShiftCircularMean1_5;
              break;
            }

          case MC_REG_ASYNC_UARTA:
          case MC_REG_ASYNC_UARTB:
          case MC_REG_ASYNC_STLNK:
          default:
          {
            retVal = MCP_ERROR_UNKNOWN_REG;
            break;
          }
        }

        /* Size of the answer is size of the data + 2 bytes containing data size */
        *size = (*rawSize) + 2U;
        break;
      }

      default:
      {
        retVal = MCP_ERROR_BAD_DATA_TYPE;
        break;
      }
    }
    return (retVal);
  }

uint8_t RI_MovString(const char_t *srcString, char_t *destString, uint16_t *size, int16_t maxSize)
{
  uint8_t retVal = MCP_CMD_OK;
  const char_t *tempsrcString = srcString;
  char_t *tempdestString = destString;
  *size= 1U ; /* /0 is the min String size */

  while ((*tempsrcString != (char_t)0) && (*size < (uint16_t)maxSize))
  {
    *tempdestString = *tempsrcString;
    tempdestString++;
    tempsrcString++;
    *size = *size + 1U;
  }

  if (*tempsrcString != (char_t)0)
  { /* Last string char must be 0 */
    retVal = MCP_ERROR_STRING_FORMAT;
  }
  else
  {
    *tempdestString = (int8_t)0;
  }
  return (retVal);
}

uint8_t RI_GetIDSize(uint16_t dataID)
{
  uint8_t typeID = ((uint8_t)dataID) & TYPE_MASK;
  uint8_t result;

  switch (typeID)
  {
    case TYPE_DATA_8BIT:
    {
      result = 1;
      break;
    }

    case TYPE_DATA_16BIT:
    {
      result = 2;
      break;
    }

    case TYPE_DATA_32BIT:
    {
      result = 4;
      break;
    }

    default:
    {
      result=0;
      break;
    }
  }

  return (result);
}

__weak uint8_t RI_GetPtrReg(uint16_t dataID, void **dataPtr)
{

  uint8_t retVal = MCP_CMD_OK;
  static uint16_t nullData16=0;

#ifdef NULL_PTR_CHECK_REG_INT
  if (MC_NULL == dataPtr)
  {
    retVal = MCP_CMD_NOK;
  }
  else
  {
#endif

    MCI_Handle_t *pMCIN = &Mci[0];
    uint16_t regID = dataID & REG_MASK;
    uint8_t typeID = ((uint8_t)dataID) & TYPE_MASK;

    switch (typeID)
    {
      case TYPE_DATA_16BIT:
      {
        switch (regID)
        {
          case MC_REG_I_A:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iab.a);
             break;
          }

          case MC_REG_I_B:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iab.b);
            break;
          }

          case MC_REG_I_ALPHA_MEAS:
          {
            *dataPtr = &(pMCIN->pFOCVars->Ialphabeta.alpha);
            break;
          }

          case MC_REG_I_BETA_MEAS:
          {
            *dataPtr = &(pMCIN->pFOCVars->Ialphabeta.beta);
            break;
          }

          case MC_REG_I_Q_MEAS:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iqd.q);
            break;
          }

          case MC_REG_I_D_MEAS:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iqd.d);
            break;
          }

          case MC_REG_I_Q_REF:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iqdref.q);
            break;
          }

          case MC_REG_I_D_REF:
          {
            *dataPtr = &(pMCIN->pFOCVars->Iqdref.d);
            break;
          }

          case MC_REG_V_Q:
          {
            *dataPtr = &(pMCIN->pFOCVars->Vqd.q);
            break;
          }

          case MC_REG_V_D:
          {
            *dataPtr = &(pMCIN->pFOCVars->Vqd.d);
            break;
          }

          case MC_REG_V_ALPHA:
          {
            *dataPtr = &(pMCIN->pFOCVars->Valphabeta.alpha);
            break;
          }

          case MC_REG_V_BETA:
          {
            *dataPtr = &(pMCIN->pFOCVars->Valphabeta.beta);
            break;
          }

          case MC_REG_HALL_SPEED:
          {
            *dataPtr = &((&HALL_M1)->_Super.hAvrMecSpeedUnit);
            break;
          }

          case MC_REG_HALL_EL_ANGLE:
          {
            *dataPtr = &((&HALL_M1)->_Super.hElAngle);
            break;
          }

          case MC_REG_STOPLL_ROT_SPEED:
          {
            *dataPtr = &((&STO_PLL_M1)->_Super.hAvrMecSpeedUnit);
            break;
          }

          case MC_REG_STOPLL_EL_ANGLE:
          {
            *dataPtr = &((&STO_PLL_M1)->_Super.hElAngle);
            break;
          }

#ifdef NOT_IMPLEMENTED /* Not yet implemented */
          case MC_REG_STOPLL_I_ALPHA:
          case MC_REG_STOPLL_I_BETA:
            break;
#endif

          case MC_REG_STOPLL_BEMF_ALPHA:
          {
            *dataPtr = &((&STO_PLL_M1)->hBemf_alfa_est);
            break;
          }

          case MC_REG_STOPLL_BEMF_BETA:
          {
            *dataPtr = &((&STO_PLL_M1)->hBemf_beta_est);
            break;
          }

          default:
          {
            *dataPtr = &nullData16;
            retVal = MCP_ERROR_UNKNOWN_REG;
            break;
          }
        }
        break;
      }

      default:
      {
        *dataPtr = &nullData16;
        retVal = MCP_ERROR_UNKNOWN_REG;
        break;
      }
    }
#ifdef NULL_PTR_CHECK_REG_INT
  }
#endif
  return (retVal);
}
/************************ (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
