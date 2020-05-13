/*
TTP229 block
*/
  
//% weight=8 color=#FFD700  block="TTP229"
namespace TTP229
{

     //% blockId=TTP229_GetKeyCode
     //% block="SODPIN %SDO|SCLPIN %SCL"
     export function GetKeyCode(SDO: DigitalPin, SCL: DigitalPin): string
     {
 
          let DATA = 0;

          pins.digitalWritePin(SDO, 1);
          control.waitMicros(93);

          pins.digitalWritePin(SDO, 0);
          control.waitMicros(10);

          for (let i = 0; i < 16; i++)
          {

               pins.digitalWritePin(SCL, 1);
               pins.digitalWritePin(SCL, 0);

               DATA |= pins.digitalReadPin(SDO) << i;   
          }

          control.waitMicros(2 * 1000);

          switch (DATA&0xFFFF)
          {     
               case 0xFFFE: return "1"; 
               case 0xFFFD: return "2"; 
               case 0xFFFB: return "3"; 
               case 0xFFEF: return "4";  
               case 0xFFDF: return "5"; 
               case 0xFFBF: return "6"; 
               case 0xFEFF: return "7"; 
               case 0xFDFF: return "8"; 
               case 0xFBFF: return "9"; 
               case 0xDFFF: return "0"; 
               case 0xFFF7: return "A"; 
               case 0xFF7F: return "B"; 
               case 0xF7FF: return "C"; 
               case 0x7FFF: return "D"; 
               case 0xEFFF: return "*"; 
               case 0xBFFF: return "#";      

               default: return " ";
          }          
     
     }
}

