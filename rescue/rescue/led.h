void led_port_config()
{
	DDRJ = DDRJ | 0xFF;
	PORTJ = PORTJ & 0x00;
}

void red_led()
{
	PORTJ = 0x40;
}

void blue_led()
{
	PORTJ = 0x20;
}

void green_led()
{
	PORTJ = 0x10;
}

void turn_off_led()
{
	PORTJ = 0x00;
}