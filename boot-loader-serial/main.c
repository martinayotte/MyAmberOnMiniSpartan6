

#include "stdio.h"
#include "stdarg.h"


/* Defines */
#define PAD_RIGHT 1
#define PAD_ZERO  2

/* the following should be enough for 32 bit int */
#define PRINT_BUF_LEN 16

inline int _div(int u, int b)
{
   return u / b;
}


inline void outbyte(char** dst, char c)
{
/* Note the standard number for stdout in Unix is 1 but
   I use 0 here, because I don't support stdin or stderr
*/
if (*dst)
    *(*dst)++ = c;
else
//    _outbyte(c);
    putchar(c);
}


int SPrintf(char* dst, const char *fmt, ...)
{
    register unsigned long *varg = (unsigned long *)(&fmt);
    *varg++;
   
    /* Need to pass a pointer to a pointer to the location to
       write the character, to that the pointer to the location
       can be incremented by the final outpute function
    */   
   
    return print(&dst, fmt, varg);
}

int Printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    char *arg1 = va_arg(args, char *);
    char *arg2 = va_arg(args, char *);
    printf(">> debug va_list args=%p\n", args);
    printf(">> debug va_arg arg1=%p\n", arg1);
    printf(">> debug *arg1=%s\n", arg1);
    printf(">> debug va_arg arg2=%p\n", arg2);
    printf(">> debug *arg2=%s\n", arg2);
    printf(">> debug fmt=%p %s\n", &fmt, fmt);
    register unsigned long *varg = (unsigned long *)(&fmt);
    printf(">> debug varg=%p\n", varg);
    printf(">> debug *varg=%p %s\n", (unsigned long *)*varg, (char *)*varg);
    *varg++;
    printf(">> debug1 *varg=%p %s\n", (unsigned long *)*varg, (char *)*varg);
    register unsigned long *varg2 = (unsigned long *)((&fmt) + 1); //sizeof(fmt));
    printf(">> debug2 *varg2=%p\n", varg2);
    printf(">> debug2 *varg2=%p %s\n", (unsigned long *)*varg2, (char *)*varg2);
    register unsigned long *varg3 = (unsigned long *)((&fmt) + 2); //sizeof(fmt));
    printf(">> debug3 *varg3=%p\n", varg3);
    printf(">> debug3 *varg3=%p %s\n", (unsigned long *)*varg3, (char *)*varg3);

    char *dst = 0;
    return print((char**)&dst, fmt, varg);
}

/*  printf supports the following types of syntax ---
    char *ptr = "Hello world!";
    char *np = 0;
    int i = 5;
    unsigned int bs = sizeof(int)*8;
    int mi;
    mi = (1 << (bs-1)) + 1;

    printf("%s\n", ptr);
    printf("printf test\n");
    printf("%s is null pointer\n", np);
    printf("%d = 5\n", i);
    printf("%d = - max int\n", mi);
    printf("char %c = 'a'\n", 'a');
    printf("hex %x = ff\n", 0xff);
    printf("hex %02x = 00\n", 0);
    printf("signed %d = unsigned %u = hex %x\n", -3, -3, -3);
    printf("%d %s(s)%", 0, "message");
    printf("\n");
    printf("%d %s(s) with %%\n", 0, "message");

*/

int print(char** dst, const char *format, unsigned long long *varg)
{
    register int width, pad;
    register int pc = 0;
    char scr[2];
     
    for (; *format != 0; ++format) {
       if (*format == '%') {
          ++format;
          width = pad = 0;
          if (*format == '\0') break;
          if (*format == '%') goto out;
          if (*format == '-') {
             ++format;
             pad = PAD_RIGHT;
          }
          while (*format == '0') {
             ++format;
             pad |= PAD_ZERO;
          }
          for ( ; *format >= '0' && *format <= '9'; ++format) {
             width *= 10;
             width += *format - '0';
          }
          if( *format == 's' ) {
             printf("--> debugA varg=%p %s\n", varg, *((char **)varg++));
             register char *s = *((char **)varg++);
             printf("--> debugB varg++=%p\n", varg);
             printf("--> debug s=%p\n", s);
             printf("--> debug *s=%s\n", s);
             pc += prints (dst, s?s:"(null)", width, pad);
             continue;
          }
          if( *format == 'd' ) {
             pc += printi (dst, *varg++, 10, 1, width, pad, 'a');
             continue;
          }
          if( *format == 'x' ) {
             pc += printi (dst, *varg++, 16, 0, width, pad, 'a');
             continue;
          }
          if( *format == 'X' ) {
             pc += printi (dst, *varg++, 16, 0, width, pad, 'A');
             continue;
          }
          if( *format == 'u' ) {
             pc += printi (dst, *varg++, 10, 0, width, pad, 'a');
             continue;
          }
          if( *format == 'c' ) {
             /* char are converted to int then pushed on the stack */
             scr[0] = *varg++;
             scr[1] = '\0';
             pc += prints (dst, scr, width, pad);
             continue;
          }
       }
       else {
       out:
          if (*format=='\n') outbyte(dst,'\r');
          outbyte (dst, *format);
          ++pc;
       }
    }

    return pc;
}


/* Print a string - no formatting characters will be interpreted here */
int prints(char** dst, const char *string, int width, int pad)
{
    register int pc = 0, padchar = ' ';

    printf("debug string=%s width=%d\n", string, width);
    
    if (width > 0) {                          
       register int len = 0;                  
       register const char *ptr;              
       for (ptr = string; *ptr; ++ptr) ++len; 
       if (len >= width) width = 0;           
       else width -= len;                     
       if (pad & PAD_ZERO) padchar = '0';     
    }                                         
    if (!(pad & PAD_RIGHT)) {                 
       for ( ; width > 0; --width) {          
          outbyte(dst, padchar);              
          ++pc;                               
       }                                      
    }                                         
    for ( ; *string ; ++string) {
       outbyte(dst, *string);                 
       ++pc;                                  
    }                                         
    for ( ; width > 0; --width) {             
       outbyte(dst, padchar);                 
       ++pc;                                  
    }                                         

    return pc;                                
}


/* Printf an integer */
int printi(char** dst, int i, int b, int sg, int width, int pad, int letbase)
{
    char print_buf[PRINT_BUF_LEN];
    char *s;
    int t, neg = 0, pc = 0;
    unsigned int u = i;

    if (i == 0) {
       print_buf[0] = '0';
       print_buf[1] = '\0';
       return prints (dst, print_buf, width, pad);
    }

    if (sg && b == 10 && i < 0) {
       neg = 1;
       u = -i;
    }

    s = print_buf + PRINT_BUF_LEN-1;
    *s = '\0';

    while (u) {
       if ( b == 16 )    t = u & 0xf;                  /* hex modulous */
       else              t = u - ( _div (u, b) * b );  /* Modulous */
       
       if( t >= 10 )
          t += letbase - '0' - 10;
       *--s = t + '0';
       
    /*   u /= b;  */
       if ( b == 16)  u = u >> 4;    /* divide by 16 */
       else           u = _div(u, b);
    }

    if (neg) {
       if( width && (pad & PAD_ZERO) ) {
          /* _outbyte('-'); */
          outbyte(dst,'-'); 
          ++pc;
          --width;
       }
       else {
          *--s = '-';
       }
    }

    return pc + prints (dst, s, width, pad);
}



void main(void) 
{
  char *str = "01234";
  int	i = 1234;
  int   u = 50000;
  
  Printf("Tourlou !\n");
//  Printf("int=%d\n", i);
  printf(">>> debug str=%p\n", str); 
  Printf("str=%s %s\n", str, str);
//  Printf("uint=%u\n", u);
}

