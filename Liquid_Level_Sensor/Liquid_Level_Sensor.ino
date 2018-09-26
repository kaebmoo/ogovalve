
/***************************************************

* Liquid Level Sensor-XKC-Y25-T12V

* ****************************************************

* This example is to get liquid level



* @author jackli(Jack.li@dfrobot.com)

* @version  V1.0

* @date  2016-1-30



* GNU Lesser General Public License.

* See <http://www.gnu.org/licenses/> for details.

* All above must be included in any redistribution

* ****************************************************/

/*
 *
 *
 * modify by P. Nivatyakul
 * github.com/kaebmoo
 * @date 2018 09 25
 *
 * first version test on Arduino UNO
 *
 */

#define WEMOS

#ifdef UNO
  #define RELAY1  7
  #define sensor0 5
  #define sensor1 6
#endif

#ifdef WEMOS
  #define RELAY1  D1
  #define sensor0 D5
  #define sensor1 D6
#endif


int Liquid_level0 = 0;
int Liquid_level1 = 0;

void setup() {

 Serial.begin(9600);

 pinMode(sensor0,INPUT);
 pinMode(sensor1,INPUT);
 pinMode(LED_BUILTIN, OUTPUT);
 pinMode(RELAY1, OUTPUT);

}


void loop() {

  Liquid_level0 = digitalRead(sensor0);
  Liquid_level1 = digitalRead(sensor1);

  Serial.print("Liquid level 0 = ");
  Serial.print(Liquid_level0,DEC);
  Serial.print("\t");
  Serial.print("Liquid level 1 = ");
  Serial.println(Liquid_level1,DEC);

  delay(1000);

  if (Liquid_level0 == 0 && Liquid_level1 == 0) {
    relayOn();
    Serial.println("Pump On");
  }
  else if (Liquid_level0 == 1 && Liquid_level1 == 1) {
    relayOff();
    Serial.println("Pump Off");
  }
}

void relayOn()
{
  digitalWrite(RELAY1, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);
}


void relayOff()
{
  digitalWrite(RELAY1, LOW);
  digitalWrite(LED_BUILTIN, LOW);
}
