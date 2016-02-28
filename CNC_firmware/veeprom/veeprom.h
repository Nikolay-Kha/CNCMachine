#ifndef _V_EEPROM_H
#define _V_EEPROM_H

int VEEPROM_GetData(unsigned int page, void *out, unsigned int size);
int VEEPROM_SetData(unsigned int page, void *data, unsigned int size);


#endif //_V_EEPROM_H
