
const unsigned char ps2_to_sun_tbl[128] = {
	0x0 , //  0 (0) : 
	0x12, //  1 (1) : F9
	0x0 , //  2 (2) : 
	0x0C, //  3 (3) : F5
	0x08, //  4 (4) : F3
	0x05, //  5 (5) : F1
	0x06, //  6 (6) : F2
	0x01, //  7 (7) : F12 => STOP (F12=0x0B)
	0x0 , //  8 (8) : 
	0x07, //  9 (9) : F10
	0x11, //  A (10): F8
	0x0E, //  B (11): F6
	0x0A, //  C (12): F4
	0x35, //  D (13): TAB
	0x2A, //  E (14): `  (FR: ^2)
	0x0 , //  F (15): 
	0x0 , // 10 (16): 
	0x13, // 11 (17): LALT
	0x6E, // 12 (18): LSHIFT
	0x0 , // 13 (19): 
	0x4C, // 14 (20): LCTRL
	0x36, // 15 (21): Q
	0x1E, // 16 (22): 1
	0x0 , // 17 (23): 
	0x0 , // 18 (24): 
	0x0 , // 19 (25): 
	0x64, // 1A (26): Z
	0x4E, // 1B (27): S
	0x4D, // 1C (28): A
	0x37, // 1D (29): W
	0x1F, // 1E (30): 2
	0x0 , // 1F (31): 
	0x0 , // 20 (): 
	0x66, // 21 (): C
	0x65, // 22 (): X
	0x4F, // 23 (): D
	0x38, // 24 (): E
	0x21, // 25 (): 4
	0x20, // 26 (): 3
	0x0 , // 27 (): 
	0x0 , // 28 (): 
	0x79, // 29 (): SPACE
	0x67, // 2A (): V
	0x50, // 2B (): F
	0x3A, // 2C (): T
	0x39, // 2D (): R
	0x22, // 2E (): 5
	0x0 , // 2F (): 
	0x0 , // 30 () : 
	0x69, // 31 (1) : N
	0x68, // 32 (2) : B
	0x52, // 33 (3) : H
	0x51, // 34 (4) : G
	0x3B, // 35 (5) : Y
	0x23, // 36 (6) : 6
	0x0 , // 37 (7) : 
	0x0 , // 38 (8) : 
	0x0 , // 39 (9) : 
	0x6A, // 3A (10):    (FR: ,)
	0x53, // 3B (11): J
	0x3C, // 3C (12): U
	0x24, // 3D (13): 7
	0x25, // 3E (14): 8
	0x0 , // 3F (15): 
	0x0 , // 40 (0) : 
	0x6B, // 41 (1) : ,  (FR: ;)
	0x54, // 42 (2) : K
	0x3D, // 43 (3) : I
	0x3E, // 44 (4) : O
	0x27, // 45 (5) : 0 (zero)
	0x26, // 46 (6) : 9
	0x0 , // 47 (7) : 
	0x0 , // 48 (8) : 
	0x6C, // 49 (9) : .
	0x6D, // 4A (10): SLASH   (FR: !)
	0x55, // 4B (11): L
	0x56, // 4C (12): M
	0x3F, // 4D (13): P
	0x28, // 4E (14):        (FR: ))
	0x0 , // 4F (15): 
	0x0 , // 50 (0) : 
	0x0 , // 51 (1) : 
	0x57, // 52 (2) :        (FR: ù)
	0x0 , // 53 (3) : 
	0x40, // 54 (4) : [
	0x29, // 55 (5) : =
	0x0 , // 56 (6) : 
	0x0 , // 57 (7) : 
	0x77, // 58 (8) : CAPS
	0x6E, // 59 (9) : RSHIFT
	0x59, // 5A (10): RETURN
	0x41, // 5B (11): ]
	0x0 , // 5C (12): 
	0x58, // 5D (13): CSLASH   (FR: *)
	0x0 , // 5E (14): 
	0x0 , // 5F (15): 
	0x0 , // 60 (0) : 
	0x7C, // 61 (1) :    (FR: <)
	0x0 , // 62 (2) : 
	0x0 , // 63 (3) : 
	0x0 , // 64 (4) : 
	0x0 , // 65 (5) : 
	0x2B, // 66 (6) : BKSP
	0x0 , // 67 (7) : 
	0x0 , // 68 (8) : 
	0x70, // 69 (9) : KP 1
	0x0 , // 6A (10): 
	0x5B, // 6B (11): KP 4
	0x44, // 6C (12): KP 7
	0x0 , // 6D (13): 
	0x0 , // 6E (14): 
	0x0 , // 6F (15): 
	0x5E, // 70 (0) : KP 0
	0x32, // 71 (1) : KP .
	0x71, // 72 (2) : KP 2
	0x5C, // 73 (3) : KP 5
	0x5D, // 74 (4) : KP 6
	0x45, // 75 (5) : KP 8
	0x1D, // 76 (6) : ESC
	0x62, // 77 (7) : NUM
	0x09, // 78 (8) : F11
	0x7D, // 79 (9) : KP+
	0x72, // 7A (10): KP 3
	0x47, // 7B (11): KP -
	0x2F, // 7C (12): KP *
	0x46, // 7D (13): KP 9
	0x0 , // 7E (14): SCROLL
	0x0 , // 7F (15): 
};

const unsigned char ps2_ext_to_sun_tbl[128] = {
	0x0 , //  00 : 
	0x0 , //  01 : 
	0x0 , //  02 : 
	0x0 , //  03 : 
	0x0 , //  04 : 
	0x0 , //  05 : 
	0x0 , //  06 : 
	0x0 , //  07 : 
	0x0 , //  08 : 
	0x0 , //  09 : 
	0x0 , //  0A : 
	0x0 , //  0B : 
	0x0 , //  0C : 
	0x0 , //  0D : 
	0x0 , //  0E : 
	0x0 , //  0F : 
	0x0 , //  10 : 
	0x0D, //  11 : RALT
	0x0 , //  12 : 
	0x0 , //  13 : 
	0x4C, //  14 : RCTRL
	0x0 , //  15 : 
	0x0 , //  16 : 
	0x0 , //  17 : 
	0x0 , //  18 : 
	0x0 , //  19 : 
	0x0 , //  1A : 
	0x0 , //  1B : 
	0x0 , //  1C : 
	0x0 , //  1D : 
	0x0 , //  1E : 
	0x78, //  1F : LGUI => L triangle
	0x0 , //  20 : 
	0x0 , //  21 : 
	0x0 , //  22 : 
	0x0 , //  23 : 
	0x0 , //  24 : 
	0x0 , //  25 : 
	0x0 , //  26 : 
	0x7A, //  27 : RGUI => R triangle
	0x0 , //  28 : 
	0x0 , //  29 : 
	0x0 , //  2A : 
	0x0 , //  2B : 
	0x0 , //  2C : 
	0x0 , //  2D : 
	0x0 , //  2E : 
	0x0 , //  2F : APPS
	0x0 , //  30 : 
	0x0 , //  31 : 
	0x0 , //  32 : 
	0x0 , //  33 : 
	0x0 , //  34 : 
	0x0 , //  35 : 
	0x0 , //  36 : 
	0x0 , //  37 : 
	0x0 , //  38 : 
	0x0 , //  39 : 
	0x0 , //  3A : 
	0x0 , //  3B : 
	0x0 , //  3C : 
	0x0 , //  3D : 
	0x0 , //  3E : 
	0x0 , //  3F : 
	0x0 , //  40 : 
	0x0 , //  41 : 
	0x0 , //  42 : 
	0x0 , //  43 : 
	0x0 , //  44 : 
	0x0 , //  45 : 
	0x0 , //  46 : 
	0x0 , //  47 : 
	0x0 , //  48 : 
	0x0 , //  49 : 
	0x2E, //  4A : KP /
	0x0 , //  4B : 
	0x0 , //  4C : 
	0x0 , //  4D : 
	0x0 , //  4E : 
	0x0 , //  4F : 
	0x0 , //  50 : 
	0x0 , //  51 : 
	0x0 , //  52 : 
	0x0 , //  53 : 
	0x0 , //  54 : 
	0x0 , //  55 : 
	0x0 , //  56 : 
	0x0 , //  57 : 
	0x0 , //  58 : 
	0x0 , //  59 : 
	0x5A, //  5A : KP ENTER
	0x0 , //  5B : 
	0x0 , //  5C : 
	0x0 , //  5D : 
	0x0 , //  5E : 
	0x0 , //  5F : 
	0x0 , //  60 : 
	0x0 , //  61 : 
	0x0 , //  62 : 
	0x0 , //  63 : 
	0x0 , //  64 : 
	0x0 , //  65 : 
	0x0 , //  66 : 
	0x0 , //  67 : 
	0x0 , //  68 : 
	0x4A, //  69 : end
	0x0 , //  6A : 
	0x18, //  6B : left arrow
	0x34, //  6C : home
	0x0 , //  6D : 
	0x0 , //  6E : 
	0x0 , //  6F : 
	0x2C, //  70 : insert
	0x32, //  71 : DELETE
	0x1B, //  72 : down arrow
	0x0 , //  73 : 
	0x1C, //  74 : right arrow
	0x14, //  75 : up arrow
	0x0 , //  76 : 
	0x0 , //  77 :
	0x0 , //  78 : 
	0x0 , //  79 : 
	0x7B, //  7A : page down
	0x0 , //  7B : 
	0x0 , //  7C : 
	0x60, //  7D : page up
	0x0 , //  7E : 
	0x0 , //  7F : 
};

// 0x0E, 83 : F7

// https://kentie.net/article/sunkbd/page2.htm