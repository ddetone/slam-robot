void spi_printdata(void) {
  spi_transfer_str("!");

#if PRINT_EULER == 1
  spi_transfer_str("ANG:");
  spi_transfer_double(ToDeg(roll));
  spi_transfer_str(",");
  spi_transfer_double(ToDeg(pitch));
  spi_transfer_str(",");
  spi_transfer_double(ToDeg(yaw));
#endif      
#if PRINT_ANALOGS==1
  spi_transfer_str(",AN:");
  spi_transfer_int(AN[sensors[0]]);
  spi_transfer_str(",");
  spi_transfer_int(AN[sensors[1]]);
  spi_transfer_str(",");
  spi_transfer_int(AN[sensors[2]]);
  spi_transfer_str(",");
  spi_transfer_int(ACC[0]);
  spi_transfer_str(",");
  spi_transfer_int(ACC[1]);
  spi_transfer_str(",");
  spi_transfer_int(ACC[2]);
  spi_transfer_str(",");
  spi_transfer_int(magnetom_x);
  spi_transfer_str(",");
  spi_transfer_int(magnetom_y);
  spi_transfer_str(",");
  spi_transfer_int(magnetom_z);
#endif

  spi_println();
}

void printdata(void)
{    
      Serial.print("!");
      
      #if PRINT_EULER == 1
      Serial.print("ANG:");
      Serial.print(ToDeg(roll));
      Serial.print(",");
      Serial.print(ToDeg(pitch));
      Serial.print(",");
      Serial.print(ToDeg(yaw));
      #endif      
      #if PRINT_ANALOGS==1
      Serial.print(",AN:");
      Serial.print(AN[sensors[0]]);  //(int)read_adc(0)
      Serial.print(",");
      Serial.print(AN[sensors[1]]);
      Serial.print(",");
      Serial.print(AN[sensors[2]]);  
      Serial.print(",");
      Serial.print(ACC[0]);
      Serial.print (",");
      Serial.print(ACC[1]);
      Serial.print (",");
      Serial.print(ACC[2]);
      Serial.print(",");
      Serial.print(magnetom_x);
      Serial.print (",");
      Serial.print(magnetom_y);
      Serial.print (",");
      Serial.print(magnetom_z);      
      #endif
      /*#if PRINT_DCM == 1
      Serial.print (",DCM:");
      Serial.print(convert_to_dec(DCM_Matrix[0][0]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[0][1]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[0][2]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[1][0]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[1][1]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[1][2]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[2][0]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[2][1]));
      Serial.print (",");
      Serial.print(convert_to_dec(DCM_Matrix[2][2]));
      #endif*/
      Serial.println();
}

long convert_to_dec(float x)
{
  return x*10000000;
}

