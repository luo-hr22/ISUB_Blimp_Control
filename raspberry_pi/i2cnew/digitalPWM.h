#ifdef __cplusplus

extern "C" {

#endif



extern int  digitalPwmCreate (int pin, int value, int range) ;

extern void digitalPwmWrite  (int pin, int value) ;

extern void digitalPwmStop   (int pin) ;



#ifdef __cplusplus

}

#endif