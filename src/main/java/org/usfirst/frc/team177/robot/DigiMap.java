package org.usfirst.frc.team177.robot;

public class DigiMap {

	public static final byte[] OSC = { (byte) 0x21 };
	public static final byte[] BLINK = { (byte) 0x81 };
	public static final byte[] BRIGHT = { (byte) 0xEF };

	private static int getIndex(char value)  {
		char chr = value;
		if (chr < 32 || chr > 127) // outside displayable ascii characters
			chr = ' ';
		int index = chr - ' ';  // use ' ' to offset to 0
		return index;
	}
	
	public static byte getRawLower(char value) {
		return BYTE_MAP[getIndex(value)][0];
	}
	
	public static byte getRawUpper(char value) {
		return BYTE_MAP[getIndex(value)][1];
	}

	public static byte[] getRawDisplay(char value) {
		return BYTE_MAP[getIndex(value)];
	}

	
	// Maps numbers, letters, symbols to the 12 segment display
	private static final byte[][] BYTE_MAP = 
		{   { (byte) 0b00000000, (byte) 0b00000000 }, //
			{ (byte) 0b00000110, (byte) 0b00000000 }, // !
			{ (byte) 0b00100000, (byte) 0b00000010 }, // "
			{ (byte) 0b11001110, (byte) 0b00010010 }, // #
			{ (byte) 0b11101101, (byte) 0b00010010 }, // $
			{ (byte) 0b00100100, (byte) 0b00100100 }, // %
			{ (byte) 0b01011101, (byte) 0b00001011 }, // &
			{ (byte) 0b00000000, (byte) 0b00000100 }, // '
			{ (byte) 0b00000000, (byte) 0b00001100 }, // (
			{ (byte) 0b00000000, (byte) 0b00100001 }, // )
			{ (byte) 0b11000000, (byte) 0b00111111 }, // *
			{ (byte) 0b11000000, (byte) 0b00010010 }, // +
			{ (byte) 0b00000000, (byte) 0b00100000 }, // ,
			{ (byte) 0b11000000, (byte) 0b00000000 }, // -
			{ (byte) 0b00000000, (byte) 0b00000000 }, // .
			{ (byte) 0b00000000, (byte) 0b00100100 }, // /
			{ (byte) 0b00111111, (byte) 0b00100100 }, // 0
			{ (byte) 0b00000110, (byte) 0b00000000 }, // 1
			{ (byte) 0b11011011, (byte) 0b00000000 }, // 2
			{ (byte) 0b10001111, (byte) 0b00000000 }, // 3
			{ (byte) 0b11100110, (byte) 0b00000000 }, // 4
			{ (byte) 0b01101001, (byte) 0b00001000 }, // 5
			{ (byte) 0b11111101, (byte) 0b00000000 }, // 6
			{ (byte) 0b00000111, (byte) 0b00000000 }, // 7
			{ (byte) 0b11111111, (byte) 0b00000000 }, // 8
			{ (byte) 0b11101111, (byte) 0b00000000 }, // 9
			{ (byte) 0b00000000, (byte) 0b00010010 }, // :
			{ (byte) 0b00000000, (byte) 0b00100010 }, // ;
			{ (byte) 0b00000000, (byte) 0b00001100 }, // <
			{ (byte) 0b11001000, (byte) 0b00000000 }, // =
			{ (byte) 0b00000000, (byte) 0b00100001 }, // >
			{ (byte) 0b10000011, (byte) 0b00010000 }, // ?
			{ (byte) 0b10111011, (byte) 0b00000010 }, // @
			{ (byte) 0b11110111, (byte) 0b00000000 }, // A
			{ (byte) 0b10001111, (byte) 0b00010010 }, // B
			{ (byte) 0b00111001, (byte) 0b00000000 }, // C
			{ (byte) 0b00001111, (byte) 0b00010010 }, // D
			{ (byte) 0b11111001, (byte) 0b00000000 }, // E
			{ (byte) 0b01110001, (byte) 0b00000000 }, // F
			{ (byte) 0b10111101, (byte) 0b00000000 }, // G
			{ (byte) 0b11110110, (byte) 0b00000000 }, // H
			{ (byte) 0b00000000, (byte) 0b00010010 }, // I
			{ (byte) 0b00011110, (byte) 0b00000000 }, // J
			{ (byte) 0b01110000, (byte) 0b00001100 }, // K
			{ (byte) 0b00111000, (byte) 0b00000000 }, // L
			{ (byte) 0b00110110, (byte) 0b00000101 }, // M
			{ (byte) 0b00110110, (byte) 0b00001001 }, // N
			{ (byte) 0b00111111, (byte) 0b00000000 }, // O
			{ (byte) 0b11110011, (byte) 0b00000000 }, // P
			{ (byte) 0b00111111, (byte) 0b00001000 }, // Q
			{ (byte) 0b11110011, (byte) 0b00001000 }, // R
			{ (byte) 0b11101101, (byte) 0b00000000 }, // S
			{ (byte) 0b00000001, (byte) 0b00010010 }, // T
			{ (byte) 0b00111110, (byte) 0b00000000 }, // U
			{ (byte) 0b00110000, (byte) 0b00100100 }, // V
			{ (byte) 0b00110110, (byte) 0b00101000 }, // W
			{ (byte) 0b00000000, (byte) 0b00101101 }, // X
			{ (byte) 0b00000000, (byte) 0b00010101 }, // Y
			{ (byte) 0b00001001, (byte) 0b00100100 }, // Z
			{ (byte) 0b00111001, (byte) 0b00000000 }, // [
			{ (byte) 0b00000000, (byte) 0b00001001 }, // \
			{ (byte) 0b00001111, (byte) 0b00000000 }, // ]
			{ (byte) 0b00000011, (byte) 0b00100100 }, // ^
			{ (byte) 0b00001000, (byte) 0b00000000 }, // _
			{ (byte) 0b00000000, (byte) 0b00000001 }, // `
			{ (byte) 0b01011000, (byte) 0b00010000 }, // a
			{ (byte) 0b01111000, (byte) 0b00001000 }, // b
			{ (byte) 0b11011000, (byte) 0b00000000 }, // c
			{ (byte) 0b10001110, (byte) 0b00100000 }, // d
			{ (byte) 0b01011000, (byte) 0b00100000 }, // e
			{ (byte) 0b01110001, (byte) 0b00000000 }, // f
			{ (byte) 0b10001110, (byte) 0b00000100 }, // g
			{ (byte) 0b01110000, (byte) 0b00010000 }, // h
			{ (byte) 0b00000000, (byte) 0b00010000 }, // i
			{ (byte) 0b00001110, (byte) 0b00000000 }, // j
			{ (byte) 0b00000000, (byte) 0b00011110 }, // k
			{ (byte) 0b00110000, (byte) 0b00000000 }, // l
			{ (byte) 0b11010100, (byte) 0b00010000 }, // m
			{ (byte) 0b01010000, (byte) 0b00010000 }, // n
			{ (byte) 0b11011100, (byte) 0b00000000 }, // o
			{ (byte) 0b01110000, (byte) 0b00000001 }, // p
			{ (byte) 0b10000110, (byte) 0b00000100 }, // q
			{ (byte) 0b01010000, (byte) 0b00000000 }, // r
			{ (byte) 0b10001000, (byte) 0b00001000 }, // s
			{ (byte) 0b01111000, (byte) 0b00000000 }, // t
			{ (byte) 0b00011100, (byte) 0b00000000 }, // u
			{ (byte) 0b00000100, (byte) 0b00001000 }, // v
			{ (byte) 0b00010100, (byte) 0b00101000 }, // w
			{ (byte) 0b11000000, (byte) 0b00101000 }, // x
			{ (byte) 0b00001100, (byte) 0b00001000 }, // y
			{ (byte) 0b01001000, (byte) 0b00100000 }, // z
			{ (byte) 0b01001001, (byte) 0b00100001 }, // {
			{ (byte) 0b00000000, (byte) 0b00010010 }, // |
			{ (byte) 0b10001001, (byte) 0b00001100 }, // }
			{ (byte) 0b00100000, (byte) 0b00000101 }, // ~
			{ (byte) 0b11111111, (byte) 0b00111111 }  // DEL
	};
	
	// This represents and 4 character empty string
	public static final byte[][] BLANK = 
		{   
			{ (byte) 0b00001111, (byte) 0b00001111 }, // CTRL
			{ (byte) 0b00000000, (byte) 0b00000000 }, // ' '
			{ (byte) 0b00000000, (byte) 0b00000000 }, // ' '
			{ (byte) 0b00000000, (byte) 0b00000000 }, // ' '
			{ (byte) 0b00000000, (byte) 0b00000000 }  // ' '
		};
}
