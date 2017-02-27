#include <Wire.h>
#include <LiquidCrystal_I2C.h> // knihovna z https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
 
// vytvoří objekt lcd a nastaví jeho adresu
// 0x20 a 16 zanků na 2 řádcích
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7); 
 
void setup()
{
  lcd.begin(16,2); // inicializuje displej
   
  lcd.backlight(); // zapne podsvětlení
  lcd.print("Hello world!"); // vypíše text
}
 
void loop()
{
}
