# Lab 1: Introduction to Arduino and Robot Assembly

### Objectives

* Learn how to use the Arduino Uno and the Arduino IDE.
* Construct a simple Arduino program with multiple components and the Arduino Uno
* Assemble robot and complete a simple, autonomous task. 456


### Teams
**Team 1**: Zoe, Michael

**Team 2**: Natan, Marcela, Siming


## Internal Blink

We downloaded Arduino IDE from [this link.](https://www.arduino.cc/en/Main/Software)

Using the "Blink" code in File>Examples>1.Basics> Blink of Arduino IDE, we could make the internal LED blink. 

<iframe width="560" height="315" src="https://www.youtube.com/embed/n9FKL0pYl8Y" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
 

The code to making the internal LED blink is the following:

~~~
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}
~~~
An image of the internal LED setup is the following: 

![internal LED](https://github.com/3400-17/Team-17-Prime/blob/master/images/lab1/compressed_int_led.jpg)

<img src=https://raw.githubusercontent.com/3400-17/Team-17-Prime/master/images/lab1/compressed_int_led.jpg height=400>

<img src="https://docs.google.com/uc?id=0B1r9QYTd8YNrTmRwODRBZjV1OGs">

## External Blink

The external LED was connected in series with a 1k ohm resistor from pin8 to ground on the Arduino as seen below. 

![external led](https://drive.google.com/file/d/1R3-DfPPGGs5WDM5uDWtSZj8tuaqQfaEo/view?usp=sharing)

<img src=https://github.com/3400-17/Team-17-Prime/blob/master/images/lab1/compressed_ext_led.jpg >

~~~
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(8, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(8, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(8, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}
~~~
The vidoe of the external LED blinking is shown the video below:

<iframe width="560" height="315" src="https://www.youtube.com/embed/UREEzOB6Taw" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

An image of the external LED blinking is shown in the image below:

![external LED](/images/lab1/compressed_ext_led.jpg)


## Potentiometer Serial Read

The potentiometer was used to divide voltage, and therefore control the associated electronics in the circuit. The potentiometer works as the following:

<img src="https://docs.google.com/uc?id=0B1r9QYTd8YNrTmRwODRBZjV1OGs">


We connected the potentiometer to the Arduino like so:

![potentiometer to serial port](/images/lab1/pot_serial.gif)

The circuit setup is like the following:

![potentiometer](/images/lab1/compressed_pot_print.jpg)

We then moditied the 'sensorPin' to be the zeroth pin of the analog pins, and set the sensor value to be 0. 

~~~
int sensorPin = A0;
int sensorValue=0;

void setup() {

  Serial.begin(9600);
}
~~~
We print the sensor value on the screen using a `Serial.print` call. We waited for half a second using `delay`:
~~~
// the loop function runs over and over again forever
void loop() {
  sensorValue = analogRead(sensorPin);
  Serial.print(sensorValue);
  Serial.println(" ");
  delay(1000);                       // wait for a second

}
~~~
Running the code printed out the analog value to the serial monitor. It worked as expected. Here is a video of the setup. 
<iframe width="560" height="315" src="https://www.youtube.com/embed/hpu6lJtAxwc" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

An image of screen printout and the circuit is the following:

![pot_print](/images/lab1/Webp.net-resizeimage-8.jpg)






## Potentiometer to LED

We then used the potentiometer to control the brightness the LED. As we rotated the potentiometer, the LED changed brightness:

<iframe width="560" height="315" src="https://www.youtube.com/embed/4isUPZo6t5A" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

![potentiometer to LED intensity](/images/lab1/compressed_pot_led.gif)

The code for controlling LED intensity is the following:
~~~
int sensorPin = A0;
int sensorValue=0;
void setup() {
  pinMode(9, OUTPUT);
  Serial.begin(9600);
}

// the loop function runs over and over again forever
void loop() {
  sensorValue = analogRead(sensorPin);
  Serial.print(sensorValue);
  Serial.println(" ");
  analogWrite(9, sensorValue/4);   // turn the LED on (HIGH is the voltage level)
  delay(5);                       // wait for a second
}
~~~
An image of the circuit is the following:

![pot_LED](/images/lab1/Webp.net-resizeimage-10.jpg)


## Potentiometer to Servo
We then mapped the values from the potentiometer to control the servo. The Parallax continuous rotation servo takes in values from a PWM signal ranging from 0-180.  At a value of 90, the servo is not moving; increasing toward 180 the servo speeds up in one direction, and decreasing to 0 the servo speeds up in the other direction.

![potentiometer to servo motor](/images/lab1/pot_motor.gif)

The circuit setup is shown in the following image:

![pot_servo](/images/lab1/compressed_pot_servo.jpg)
~~~
#include <Servo.h> //include the servo library
Servo myservo; 

int sensorPin = A0;
int sensorValue=0;
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  //pinMode(9, OUTPUT);
  myservo.attach(9);
  Serial.begin(9600);
  
}

// the loop function runs over and over again forever
void loop() {
  sensorValue = analogRead(sensorPin);   //read the value of the potentiometer
  sensorValue = map(sensorValue, 0, 1023, 0, 180);  //map the reading between 0 and 180
  myservo.write(sensorValue);   //sets the servo position according to the scaled value 
  delay(15);                       // wait 
}
~~~

The video is a demonostration of using the rotation of potentiometer to control the servo:

<iframe width="560" height="315" src="https://www.youtube.com/embed/AC5Cre2YxCQ" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
 


## Robot Assembly and Driving in a Square

We took cues from the lab instruction and past team websites, and we assembled our own website. The final robot is shown below. 

![assembled robot](/images/lab1/compressed_robot.jpg)


The next thing we had to think about was how we would power the necessary peripherals. To power two servos from our 5V battery pack, we spliced a USB cable such that we were able to get the individual power and ground wires. Since there is only a single 5V port on the Arduino, this served us well as a temporary hack so both servos could access 5V through connection on a breadboard. The Arduino was powered through a 9V battery, that we placed beneath the Arduino mount on the robot. With everything assembled, our robot was ready to be programmed.

To drive out robot, we wrote the code for our robot to move forward and turn right. We recorded the robot moving in a square. 
~~~
#include <Servo.h>
Servo left;
Servo right; 

int sensorPin = A0; //assign sensor to pin A0
int sensorValue=0; //set sensorvalue to 0 to initialize it

void setup() {
  left.attach(10); //attach the left servo variable to pin10
  right.attach(9);  //attach the right servo variable to pin9
  Serial.begin(9600); 
  
}

// the loop function runs over and over again forever
void loop() {
  forward(); 
  delay(2000);
  turn();
  delay(1350);
  
}

void forward() {
  left.write(95); //write a value to the servo, 90 being no movement
  right.write(85); 
  
}

void back() {
  right.write(95);
  left.write(85);
  
}

void turn() {
  right.write(95); //right servo turns faster than left, enabling the robot to turn
  left.write(95);
}
~~~
The final robot functioned as we expected, and was able to run in a square. A video of our robot working autonomously is shown below:

<iframe width="560" height="315" src="https://www.youtube.com/embed/Wx0h-h_tRTE" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>


## Work Distribution

We worked in our teams together through each of the basic arduino assignments (blinking LED, potentiometer, motor).

As we got to building the robot, team 1 focused more on the code for the robot and team 2 focused on the construction of the robot.  


