#ifndef MOTORS_H
#define MOTORS_H

uint8_t motor_values[180];

void init_Timer1(void);

void setPWMleft(uint8_t speed);
void setPWMright(uint8_t speed);

void Mleftfwd(void);
void Mleftbwd(void);
void Mleftstop(void);

void Mrightfwd(void);
void Mrightbwd(void);
void Mrightstop(void);

void accelerate(uint8_t step);
void slowdown(uint8_t step);

#endif /* MOTORS_H */ 