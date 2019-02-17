/*
 * twi.h
 *
 * Created: 14.05.2017 21:10:41
 *  Author: JackFrost
 */ 


#ifndef TWI_H_
#define TWI_H_


void twi_write(TWI_t *twiname, uint8_t *writeData,uint8_t page_adress,uint8_t Adress, uint8_t bytes,bool fixed,bool skip_address);
void twi_read(TWI_t *twiname, uint8_t *readData,uint8_t page_adress, uint8_t Adress, uint8_t bytes);

#endif /* TWI_H_ */