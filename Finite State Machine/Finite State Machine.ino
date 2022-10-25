void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
  switch (finiteStateMachine) {
    case 1:
      while () {
        irRead();
        motorMove(, );
      }
      //forward
      //reading for line
      break;
    case 2:
      irRead();
      dumbController();
      //follow line with wrong direction
      break;
    case 3:
      //no line forward x time stop when line found
      break;
    case 4:  //follow line without wrong direction 
      break;
    case 5:
      //stop when no more line and return home
      break;
    case 6:
      //all stop
      break;
    default:
      break;
  }