

void espera (int time){
		int i;

		for (i=0;i<time;i++){
		}
	}

void Bin2Ascii(unsigned short value, unsigned char *temp)
	{
	unsigned short partial , quotient, divisor,j;

	partial  = value;
	divisor  = 10000;
	*(temp)= ' ';

	for (j = 1; j < 6; j++)
	    {
		quotient=partial/divisor;

		*(temp+j)='0'+(unsigned char)quotient;
		partial-=(quotient*divisor);
		divisor/=10;

	    }
	*(temp+6)=0;
	}
