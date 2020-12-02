#line 1 "utilities\\Strings.c"






#line 1 "utilities\\Strings.h"





  

void itoa(int n, char s[]);


void reverse(char s[]);


int strlen(const char* s);


char* strncpy(char *dst, const char *src, int n);


int isspace(char c);


int isdigit(char c);


int atoi(const char *s);
#line 8 "utilities\\Strings.c"




void itoa(int n, char s[])
{
	 int i, sign;

	 if ((sign = n) < 0)   
			 n = -n;           
	 i = 0;
	 do {        
			 s[i++] = n % 10 + '0';    
	 } while ((n /= 10) > 0);      
	 if (sign < 0)
			 s[i++] = '-';
	 s[i] = '\0';
	 reverse(s);
}


int strlen(const char* s)
{
	int length = 0;
	while (*s++) {
		length++;
	}
	return length;
}





void reverse(char s[])
{
	 int i, j;
	 char c;

	 for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
			 c = s[i];
			 s[i] = s[j];
			 s[j] = c;
	 }
}


char* strncpy(char *dst, const char *src, int n)
{
	
	int i = 0;
	for (; i < n && src[i]; i++) {
		dst[i] = src[i];
	}
	
	for (int j = i; j < n; j++) {
		dst[j] = '\0';
	}
	
	return dst;
}



int isspace(char c)
{
	return (c == ' ' || c == '\n' || c == '\t' || c == '\v' || c =='\f' || c == '\r');
}


int isdigit(char c)
{
	return (c >= '0' && c <= '9');
}



int atoi(const char *s)
{
    int n, sign;
    
    while (isspace(*s))
        s++;                         
    sign = (*s == '-') ? -1 : 1;
    if (*s == '+' || *s == '-')      
        s++;
    for (n = 0; isdigit(*s); s++)
        n = 10 * n + (*s -'0');
    return sign * n;
}
