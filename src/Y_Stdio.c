/*
*	FileName:Y_Stdio.c
*	Version: 1.0	
*	Description: The file implement y_print()
*	Created On:2016-2-25
*	Modified date:
*	Author:Sky
*
*/
#include "Y_Stdio.h"

#define ZEROPAD	1		/* pad with zero */
#define SIGN	2		/* unsigned/signed long */
#define PLUS	4		/* show plus */
#define SPACE	8		/* space if plus */
#define LEFT	16		/* left justified */
#define SPECIAL	32		/* 0x */
#define SMALL	64		/* use 'abcdef' instead of 'ABCDEF' */

#define do_div(n,base) ({ \
int __res; \
__asm__("divl %4":"=a" (n),"=d" (__res):"0" (n),"1" (0),"r" (base)); \
__res; })


int vsprintf(char *buf, const char *fmt, y_va_list args);

#define is_digit(c) ((c) >= '0' && (c) <= '9')

static int skip_atoi(const char ** s){
	
	int i = 0;

	while(is_digit(**s))
		i = i * 10 + *((*s)++) - '0';
	
	return i;
}

int  y_strlen(const char *s)
{
         const char *sc;
 
         for (sc = s; *sc != '\0'; ++sc)
                 /* nothing */;
        return sc - s;
 }


static char * number(char * str, int num, int base, int size, int precision
	,int type)
{
	char c,sign,tmp[36];
	const char *digits="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
	int i;

	if (type&SMALL) digits="0123456789abcdefghijklmnopqrstuvwxyz";
	if (type&LEFT) type &= ~ZEROPAD;
	if (base<2 || base>36)
		return 0;
	c = (type & ZEROPAD) ? '0' : ' ' ;
	if (type&SIGN && num<0) {
		sign='-';
		num = -num;
	} else
		sign=(type&PLUS) ? '+' : ((type&SPACE) ? ' ' : 0);
	if (sign) size--;
	if (type&SPECIAL)
		if (base==16) size -= 2;
		else if (base==8) size--;
	i=0;
	if (num==0)
		tmp[i++]='0';
	else while (num!=0)
		tmp[i++]=digits[do_div(num,base)];
	if (i>precision) precision=i;
	size -= precision;
	if (!(type&(ZEROPAD+LEFT)))
		while(size-->0)
			*str++ = ' ';
	if (sign)
		*str++ = sign;
	if (type&SPECIAL)
		if (base==8)
			*str++ = '0';
		else if (base==16) {
			*str++ = '0';
			*str++ = digits[33];
		}
	if (!(type&LEFT))
		while(size-->0)
			*str++ = c;
	while(i<precision--)
		*str++ = '0';
	while(i-->0)
		*str++ = tmp[i];
	while(size-->0)
		*str++ = ' ';
	return str;
}

int y_printf(const char * fmt, ...){
	
	y_va_list args;
	char * printbuf;
	int n;
	
	printbuf = (char *)malloc(sizeof(char) * 1024);
	memset(printbuf,0,sizeof(char)*1024);
	y_va_start(args,fmt);//init the args point to first right of fmt.
	
	write(1,printbuf,n=y_vsprintf(printbuf, fmt, args));
	
	y_va_end(args);
	
	free(printbuf);
	return n;
}

//The vsprintf() defines in linux kernel vsprintf.c file.
int y_vsprintf(char *buf, const char *fmt, y_va_list args){
	
	int len;
	int i;
 	char * str;
 	char *s;
 	int *ip;

	int flags;        /* flags to number() */
	int field_width;    /* width of output field */
 	int precision;        /* min. # of digits for integers; max
 	                   number of chars for from string */
 	int qualifier;        /* 'h', 'l', or 'L' for integer fields */


	for (str=buf ; *fmt ; ++fmt) {

 	    if (*fmt != '%') {//deal with '%'                        
	    	*str++ = *fmt;              
 	    	continue;
		}


	//follow codes is to deal with '%' 
	
	/* process flags */
 	    flags = 0;
	    repeat:
 	    ++fmt;        /* this also skips first '%' */
 	    switch (*fmt) {
 	    	case '-': flags |= LEFT; goto repeat;
	        case '+': flags |= PLUS; goto repeat;
	        case ' ': flags |= SPACE; goto repeat;                         //判断标志位，并设置flags
	        case '#': flags |= SPECIAL; goto repeat; //'%#' is to attent us,let us kown the type of value 
	        case '0': flags |= ZEROPAD; goto repeat;
	    }



		/* get field width */
	    field_width = -1;
 	    if (is_digit(*fmt))
 	    	field_width = skip_atoi(&fmt);
 	    else if (*fmt == '*') {//get width from arg
 	            /* it's the next argument */
 	    	field_width = y_va_arg(args, int);
 	    	if (field_width < 0) {
 	        	field_width = -field_width;
 	            flags |= LEFT;
 	       	}
 	    }



		/* get the precision */
		precision = -1;
		if (*fmt == '.') {
			++fmt;	
			if (is_digit(*fmt))
				precision = skip_atoi(&fmt);
			else if (*fmt == '*') {
				/* it's the next argument */
				precision = y_va_arg(args, int);
			}
			if (precision < 0)
				precision = 0;
		}


		/* get the conversion qualifier */
		qualifier = -1;
		if (*fmt == 'h' || *fmt == 'l' || *fmt == 'L') {
			qualifier = *fmt;
			++fmt;
		}

		

		switch (*fmt) {
		case 'c'://deal with "%c"
			if (!(flags & LEFT))//deal left-alignment 
				while (--field_width > 0)
					*str++ = ' ';
			*str++ = (unsigned char) y_va_arg(args, int);
			while (--field_width > 0)
				*str++ = ' ';
			break;

		
		case 's'://deal with "%s"
			s = y_va_arg(args, char *);
			if (!s)
				s = "<NULL>";
			len = y_strlen(s);
			if (precision < 0)
				precision = len;
			else if (len > precision)
				len = precision;

			if (!(flags & LEFT))
				while (len < field_width--)
					*str++ = ' ';
			for (i = 0; i < len; ++i)
				*str++ = *s++;
			while (len < field_width--)
				*str++ = ' ';
			break;

		
		case 'o':
			str = number(str, y_va_arg(args, unsigned long), 8,
				field_width, precision, flags);
			break;

		case 'p':
			if (field_width == -1) {
				field_width = 8;
				flags |= ZEROPAD;
			}
			str = number(str,
				(unsigned long) y_va_arg(args, void *), 16,
				field_width, precision, flags);
			break;


		case 'x':
			flags |= SMALL;
		case 'X':
			str = number(str, y_va_arg(args, unsigned long), 16,
				field_width, precision, flags);
			break;

		case 'd':
		case 'i':
			flags |= SIGN;
		case 'u':
			str = number(str, y_va_arg(args, unsigned long), 10,
				field_width, precision, flags);
			break;

		case 'n':
			ip = y_va_arg(args, int *);
			*ip = (str - buf);
			break;
		
		default:
			if (*fmt != '%')
				*str++ = '%';
			if (*fmt)
				*str++ = *fmt;
			else
				--fmt;
			break;
		}
	}


	*str = '\0';
	return str-buf;


}


