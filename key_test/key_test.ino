// Membránová klávesnice 1x4

// pole s piny připojených tlačítek
const int tlacitka[] = {4,6,10,12};
// proměnná stavu tlačítka
int stisk = 0;

void setup() {
  // komunikace přes sériovou linku rychlostí 9600 baud
  Serial.begin(9600);  
  // inicializace pole tlačítek
  for(int x=0; x<4; x++)
  {
    // zapojení tlačítek jako vstup s pull-up odporem
    pinMode(tlacitka[x], INPUT_PULLUP);
  }  
}

void loop(){
  // čtení stavu jednotlivých tlačítek ve smyčce
  for(int x=0; x<4; x++)
  {
    //načtení stavu tlačítka a uložení do proměnné stisk
    stisk = digitalRead(tlacitka[x]);

    // pokud je tlačítko stisknuto
    // vykonej následující
    if (stisk == LOW) {    
      // vytiskni pres sériovou linku číslo tlačítka a zprávu
      Serial.print(x);
      Serial.println(" stisknuto");
      // vyčkej x00 ms kvůli zbytečnému množství tisknutých znaků
      delay(200);
    }
  }
}
