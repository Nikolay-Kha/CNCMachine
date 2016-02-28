#include <stdio.h>
#include <stdarg.h>


signed int vsnprintf(char *pStr, size_t length, const char *pFormat, va_list ap)
{
	char digitBuffer[32]; //32 символа на цифры
	char *pOriginalStr = pStr;
	length--; // for null terminated char

	/* Phase string */
	while (*pFormat != 0 && (pStr-pOriginalStr)<length){
		if (*pFormat != '%'){
			*pStr++ = *pFormat++;
		} else if (*(pFormat+1) == '%'){
			*pStr++ = '%';
			pFormat += 2;
		} else {
			pFormat++;

			switch (*pFormat) {
			case 'd':
			case 'i':{
				char *tmp = digitBuffer;
				int val = va_arg(ap, signed int);
				if(val<0) {
					val = -val;
					*pStr++ = '-';
				}
				do {
					*tmp++ = val % 10 + '0';
				} while ( (val /= 10) > 0);
				tmp--;
				while( (pStr-pOriginalStr)<length && tmp>=digitBuffer) *pStr++= *tmp--;

			}
			break;
			//  case 'u': num = PutUnsignedInt(pStr, fill, width, va_arg(ap, unsigned int)); break;
			//  case 'x': num = PutHexa(pStr, fill, width, 0, va_arg(ap, unsigned int)); break;
			//  case 'X': num = PutHexa(pStr, fill, width, 1, va_arg(ap, unsigned int)); break;
			case 's': {
				char * t = va_arg(ap, char *);
				while (*t && (pStr-pOriginalStr)<length)	*pStr++ = *t++;
				break;
			}
			case 'c': *pStr++ = va_arg(ap, unsigned int); break;
			case 'f':{ // .4f
				char *tmp = digitBuffer;
				double z = va_arg(ap, double);
				int val;
				if(z>=0.0f) {
					val = (z*10000.0f+0.5f);
				} else {
					val = (-z*10000.0f+0.5f);
					*pStr++ = '-';
				}
				int i = 0;
				do {
					*tmp++ = val % 10 + '0';
					i++;
					if(i==4)
						*tmp++ = '.';
				} while ( (val /= 10) > 0 || i<5);
				tmp--;
				while( (pStr-pOriginalStr)<length && tmp>=digitBuffer) *pStr++= *tmp--;
			}
			break;
			default:
				return EOF;
			}

			pFormat++;
		}
	}
	*pStr = 0;

	return (pStr-pOriginalStr);
}

signed int snprintf(char *pString, size_t length, const char *pFormat, ...)
{
    va_list    ap;
    signed int rc;

    va_start(ap, pFormat);
    rc = vsnprintf(pString, length, pFormat, ap);
    va_end(ap);

    return rc;
}
