/*
 * 2005 VOLVO XC70 steering wheel buttons interface for Kenwood head unit
 *
 * (c) 2014 Vitaly Mayatskikh vitaly@gravicappa.info
 *
 * 
 * Steering wheel module CAN ID is 0x0131726c
 * Message is 8 bytes long: 0x00 0x0c 0x28 0x52 0x80 0x00 0x00 0x3f
 *                                                                ^
 *                                                these 4 bits are audio buttons
 * bit 0 - previous track
 * bit 1 - next track
 * bit 2 - volume down
 * bit 3 - volume up
 *
 * 40 messages per second
 *
 * Kenwood codes were taken from here: http://custombaggerforum.com/forum/showthread.php?551-Road-Glide-Hacks&p=7085&viewfull=1#post7085
 * Thanks, Len Shelton!
 *
 * CCM (?) CAN ID 0x02803008
 * Sends ambient light level
 * Message is 8 bytes long: 0x8D 0x41 0x90 0xF9 0xFF 0x00 0x40 0x00
 *                                           ^^
 *                                           |Instrument cluster backlight 0..F
 *                                           Ambient light (F - day, !F - not day)
 *                            
 */

#include <SPI.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>

//#define TEST_MODE

#define SWM_CAN_ID 0x0131726c
#define CCM_CAN_ID 0x02803008

/* direct button mappings */
#define SWM_TRK_PREV    (1ULL << (8 * 7 + 0))
#define SWM_TRK_NEXT    (1ULL << (8 * 7 + 1))
#define SWM_VOL_DOWN    (1ULL << (8 * 7 + 2))
#define SWM_VOL_UP      (1ULL << (8 * 7 + 3))
/* combo mappings for extra functions */
#define COMBO_DISC_PREV (SWM_TRK_PREV | SWM_VOL_DOWN)
#define COMBO_DISC_NEXT (SWM_TRK_NEXT | SWM_VOL_UP)
//#define COMBO_ILLUMI    (SWM_TRK_PREV | SWM_VOL_UP)

#define SWC_OUTPUT    7
#define ILLUMI_OUTPUT 6
#define CAN_RESET     5

#define NR_ACTIONS 6

MCP_CAN CAN(10);

unsigned char cmd_common[] = { 0,1,0,0,0,1,0,1,0,1,0,0,1,0,0,1,0,1,0,0,0,0,1,0,0 };

struct action {
  unsigned long long mask;
  void (*on_press)(struct action *);
  void (*on_release)(struct action *);
  char msg[16];
  char opaque[32];
};

struct swc_action {
  boolean kenwood_cmd[24];
};

void swc_action_on_press(struct action *a) {
  struct swc_action *s = (struct swc_action *)a->opaque;
  Serial.println(a->msg);
  send(s->kenwood_cmd);
}

struct toggle_action {
  unsigned char pin;
  boolean last_state;
  boolean pin_state;
};

void toggle_action_on_press(struct action *a) {
  struct toggle_action *t = (struct toggle_action *)a->opaque;
  if (t->last_state == false) {
    t->last_state = true;
    t->pin_state = !t->pin_state;
    Serial.print(a->msg); Serial.println(t->pin_state);
    digitalWrite(t->pin, t->pin_state);
  }
}

void delay_on_release(struct action *a) {
  unsigned char len;
  unsigned char buf[8];

  Serial.println("delay");
  delay(1000); // give human some time to release combo button
  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    CAN.readMsgBuf(&len, buf);
  }
};

void toggle_action_on_release(struct action *a) {
  struct toggle_action *t = (struct toggle_action *)a->opaque;
  t->last_state = false;
  delay_on_release(a);
}

struct action actions[NR_ACTIONS] =
  {
//    { COMBO_ILLUMI,    toggle_action_on_press, toggle_action_on_release, "ILLUMI",   { ILLUMI_OUTPUT, false, true } },
    { COMBO_DISC_PREV, swc_action_on_press,    delay_on_release,         "DSC_PREV", { 0,0,1,0,1,0,0,0,0,0,1,0,1,0,0,0,1,0,1,0,1,0,1,0 } },
    { COMBO_DISC_NEXT, swc_action_on_press,    delay_on_release,         "DSC_NEXT", { 1,0,0,1,0,1,0,0,0,0,0,0,1,0,0,0,1,0,1,0,1,0,1,0 } },
    { SWM_TRK_PREV,    swc_action_on_press,    NULL,                     "TRK_PREV", { 0,1,0,0,1,0,0,0,0,0,1,0,0,1,0,0,1,0,1,0,1,0,1,0 } },
    { SWM_TRK_NEXT,    swc_action_on_press,    NULL,                     "TRK_NEXT", { 1,0,1,0,0,1,0,0,0,0,0,0,0,1,0,0,1,0,1,0,1,0,1,0 } },
    { SWM_VOL_DOWN,    swc_action_on_press,    NULL,                     "VOL_DOWN", { 1,0,0,1,0,0,1,0,0,0,0,0,1,0,0,1,0,0,1,0,1,0,1,0 } },
    { SWM_VOL_UP,      swc_action_on_press,    NULL,                     "VOL_UP",   { 0,0,1,0,0,1,0,0,0,0,1,0,1,0,0,1,0,0,1,0,1,0,1,0 } }
  };

struct action *last_action = NULL;

void send(boolean *command)
{
  digitalWrite(SWC_OUTPUT, LOW);
  delay(10);
  digitalWrite(SWC_OUTPUT, HIGH);
  delayMicroseconds(4500);

  for (int i = 0; i < 25; i++)
    send_bit(cmd_common[i]);
  for (int i = 0; i < 24; i++)
    send_bit(command[i]);
}

void send_bit(boolean bit)
{
  digitalWrite(SWC_OUTPUT, bit);
  delayMicroseconds(1000);
  digitalWrite(SWC_OUTPUT, HIGH);
  delayMicroseconds(200);
}

boolean can_ok = false;

void can_reset()
{
  digitalWrite(CAN_RESET, LOW);
  delay(100);
  digitalWrite(CAN_RESET, HIGH);
}

void setup()
{
  int i;
  
  Serial.begin(9600);
  Serial.println("start");

  pinMode(CAN_RESET, OUTPUT);
  digitalWrite(CAN_RESET, HIGH);
  pinMode(SWC_OUTPUT, OUTPUT);
  digitalWrite(SWC_OUTPUT, HIGH);
  pinMode(ILLUMI_OUTPUT, OUTPUT);
  digitalWrite(ILLUMI_OUTPUT, HIGH);

  Serial.println("Kenwood SWC inited");

#ifdef TEST_MODE
/* Pins 1..4 are for testing w/o CAN shield */

  for (i = 1; i <= 4; i++) {
    pinMode(i, INPUT);
    digitalWrite(i, HIGH);
  }
#else
  Serial.println("CAN BUS Shield initialize...");

  for (i = 2; i > 0; i--) {
    can_reset();
    for (int j = 4; 4 > 0; j--) {
      if (CAN.begin(CAN_125KBPS) == CAN_OK) {
        can_ok = true;
        goto done;
      }
      Serial.print(".");
      delay(250);
    }
  }
done:
  if (can_ok) {
    Serial.println("... inited!");
    Serial.println("CAN BUS Shield setting filter");
    if (CAN.init_Mask(0, 1, 0x03ffffff) != MCP2515_OK)
      Serial.println("init mask 0 failed!");
    if (CAN.init_Mask(1, 1, 0x03ffffff) != MCP2515_OK)
      Serial.println("init mask 1 failed!");
    if (CAN.init_Filt(0, 1, SWM_CAN_ID) != MCP2515_OK)
      Serial.println("init filter 0 failed!");
    if (CAN.init_Filt(1, 1, CCM_CAN_ID) != MCP2515_OK)
      Serial.println("init filter 0 failed!");
  } else {
    Serial.println("... failed!");
    return;
  }
#endif
  Serial.println("All set!");
}

void loop()
{
#ifdef TEST_MODE
  check_pins();
#else
  if (can_ok)
    check_canbus();
#endif
}

void check_pins()
{
  unsigned char buf[8] = {0x00, 0x0c, 0x28, 0x52, 0x80, 0x00, 0x00, 0x3f};

  buf[7] ^= ((!digitalRead(1)) << 0);
  buf[7] ^= ((!digitalRead(2)) << 1);
  buf[7] ^= ((!digitalRead(3)) << 2);
  buf[7] ^= ((!digitalRead(4)) << 3);
  do_actions(buf);
}

void check_canbus()
{
  unsigned char len;
  unsigned char buf[8];

  if(CAN.checkReceive() == CAN_MSGAVAIL) {
    memset(buf, 0, sizeof(buf));
    CAN.readMsgBuf(&len, buf);
    switch (CAN.getCanId()) {
      case SWM_CAN_ID:
        do_actions(buf);
        break;
      case CCM_CAN_ID:
        digitalWrite(ILLUMI_OUTPUT, ((buf[3] & 0xf0) == 0xf0) ? HIGH : LOW);
        break;
      default:
        break;
    }  
  } 
}   

void do_actions(unsigned char *buf)
{
  int i;

  for (i = 0; i < NR_ACTIONS; i++) {
    if (do_action(i, buf))
      break;
  }
  if (i == NR_ACTIONS) {  // no buttons pressed
    if (last_action) {
      if (last_action->on_release)
        last_action->on_release(last_action);
      last_action = NULL;
    }
  }
}

boolean do_action(unsigned char idx, unsigned char *buf)
{
  boolean ret = false;
  struct action *a = &actions[idx];
  unsigned char *mask = (unsigned char *)&a->mask;
  int sum = 0;

  for (int i = 0; i < 8; i++)
    sum += buf[i] & mask[i];

  if (sum == 0) {
    if (last_action && last_action != a && last_action->on_release)
      last_action->on_release(last_action);
    if (a->on_press)
      a->on_press(a);
    last_action = a;
    ret = true;
  }
  return ret;
}

