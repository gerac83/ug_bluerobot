/*
 * Copyright (C) 2011-2012 Shahbaz Youssefi <ShabbyX@gmail.com>
 *
 * This file is part of Ngin3d.
 *
 * Ngin3d is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Ngin3d is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Ngin3d.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef NGIN3D_KEYDEF_H_BY_CYCLOPS
#define NGIN3D_KEYDEF_H_BY_CYCLOPS

enum shNgin3dKey				// shNgin3dKey is an exact copy of SDLKey (of SDL version 1.2). That is, you can safely pass SDLK_* as keys instead of NGIN3D_* equivalents (of course, cast to shNgin3dKey is needed)
{
	NGIN3D_NO_KEY		= 0,
	NGIN3D_UNKNOWN		= 0,
	NGIN3D_FIRST		= 0,
	NGIN3D_BACKSPACE	= 8,
	NGIN3D_TAB		= 9,
	NGIN3D_CLEAR		= 12,
	NGIN3D_RETURN		= 13,
	NGIN3D_PAUSE		= 19,
	NGIN3D_ESCAPE		= 27,
	NGIN3D_SPACE		= 32,
	NGIN3D_EXCLAIM		= 33,
	NGIN3D_QUOTEDBL		= 34,
	NGIN3D_HASH		= 35,
	NGIN3D_DOLLAR		= 36,
	NGIN3D_AMPERSAND	= 38,
	NGIN3D_QUOTE		= 39,
	NGIN3D_LEFTPAREN	= 40,
	NGIN3D_RIGHTPAREN	= 41,
	NGIN3D_ASTERISK		= 42,
	NGIN3D_PLUS		= 43,
	NGIN3D_COMMA		= 44,
	NGIN3D_MINUS		= 45,
	NGIN3D_PERIOD		= 46,
	NGIN3D_SLASH		= 47,
	NGIN3D_0		= 48,
	NGIN3D_1		= 49,
	NGIN3D_2		= 50,
	NGIN3D_3		= 51,
	NGIN3D_4		= 52,
	NGIN3D_5		= 53,
	NGIN3D_6		= 54,
	NGIN3D_7		= 55,
	NGIN3D_8		= 56,
	NGIN3D_9		= 57,
	NGIN3D_COLON		= 58,
	NGIN3D_SEMICOLON	= 59,
	NGIN3D_LESS		= 60,
	NGIN3D_EQUALS		= 61,
	NGIN3D_GREATER		= 62,
	NGIN3D_QUESTION		= 63,
	NGIN3D_AT		= 64,
	/*
	   Skip uppercase letters
	 */
	NGIN3D_LEFTBRACKET	= 91,
	NGIN3D_BACKSLASH	= 92,
	NGIN3D_RIGHTBRACKET	= 93,
	NGIN3D_CARET		= 94,
	NGIN3D_UNDERSCORE	= 95,
	NGIN3D_BACKQUOTE	= 96,
	NGIN3D_a		= 97,
	NGIN3D_b		= 98,
	NGIN3D_c		= 99,
	NGIN3D_d		= 100,
	NGIN3D_e		= 101,
	NGIN3D_f		= 102,
	NGIN3D_g		= 103,
	NGIN3D_h		= 104,
	NGIN3D_i		= 105,
	NGIN3D_j		= 106,
	NGIN3D_k		= 107,
	NGIN3D_l		= 108,
	NGIN3D_m		= 109,
	NGIN3D_n		= 110,
	NGIN3D_o		= 111,
	NGIN3D_p		= 112,
	NGIN3D_q		= 113,
	NGIN3D_r		= 114,
	NGIN3D_s		= 115,
	NGIN3D_t		= 116,
	NGIN3D_u		= 117,
	NGIN3D_v		= 118,
	NGIN3D_w		= 119,
	NGIN3D_x		= 120,
	NGIN3D_y		= 121,
	NGIN3D_z		= 122,
	NGIN3D_DELETE		= 127,
	/* End of ASCII mapped keysyms */

	/* International keyboard syms */
	NGIN3D_WORLD_0		= 160,		/* 0xA0 */
	NGIN3D_WORLD_1		= 161,
	NGIN3D_WORLD_2		= 162,
	NGIN3D_WORLD_3		= 163,
	NGIN3D_WORLD_4		= 164,
	NGIN3D_WORLD_5		= 165,
	NGIN3D_WORLD_6		= 166,
	NGIN3D_WORLD_7		= 167,
	NGIN3D_WORLD_8		= 168,
	NGIN3D_WORLD_9		= 169,
	NGIN3D_WORLD_10		= 170,
	NGIN3D_WORLD_11		= 171,
	NGIN3D_WORLD_12		= 172,
	NGIN3D_WORLD_13		= 173,
	NGIN3D_WORLD_14		= 174,
	NGIN3D_WORLD_15		= 175,
	NGIN3D_WORLD_16		= 176,
	NGIN3D_WORLD_17		= 177,
	NGIN3D_WORLD_18		= 178,
	NGIN3D_WORLD_19		= 179,
	NGIN3D_WORLD_20		= 180,
	NGIN3D_WORLD_21		= 181,
	NGIN3D_WORLD_22		= 182,
	NGIN3D_WORLD_23		= 183,
	NGIN3D_WORLD_24		= 184,
	NGIN3D_WORLD_25		= 185,
	NGIN3D_WORLD_26		= 186,
	NGIN3D_WORLD_27		= 187,
	NGIN3D_WORLD_28		= 188,
	NGIN3D_WORLD_29		= 189,
	NGIN3D_WORLD_30		= 190,
	NGIN3D_WORLD_31		= 191,
	NGIN3D_WORLD_32		= 192,
	NGIN3D_WORLD_33		= 193,
	NGIN3D_WORLD_34		= 194,
	NGIN3D_WORLD_35		= 195,
	NGIN3D_WORLD_36		= 196,
	NGIN3D_WORLD_37		= 197,
	NGIN3D_WORLD_38		= 198,
	NGIN3D_WORLD_39		= 199,
	NGIN3D_WORLD_40		= 200,
	NGIN3D_WORLD_41		= 201,
	NGIN3D_WORLD_42		= 202,
	NGIN3D_WORLD_43		= 203,
	NGIN3D_WORLD_44		= 204,
	NGIN3D_WORLD_45		= 205,
	NGIN3D_WORLD_46		= 206,
	NGIN3D_WORLD_47		= 207,
	NGIN3D_WORLD_48		= 208,
	NGIN3D_WORLD_49		= 209,
	NGIN3D_WORLD_50		= 210,
	NGIN3D_WORLD_51		= 211,
	NGIN3D_WORLD_52		= 212,
	NGIN3D_WORLD_53		= 213,
	NGIN3D_WORLD_54		= 214,
	NGIN3D_WORLD_55		= 215,
	NGIN3D_WORLD_56		= 216,
	NGIN3D_WORLD_57		= 217,
	NGIN3D_WORLD_58		= 218,
	NGIN3D_WORLD_59		= 219,
	NGIN3D_WORLD_60		= 220,
	NGIN3D_WORLD_61		= 221,
	NGIN3D_WORLD_62		= 222,
	NGIN3D_WORLD_63		= 223,
	NGIN3D_WORLD_64		= 224,
	NGIN3D_WORLD_65		= 225,
	NGIN3D_WORLD_66		= 226,
	NGIN3D_WORLD_67		= 227,
	NGIN3D_WORLD_68		= 228,
	NGIN3D_WORLD_69		= 229,
	NGIN3D_WORLD_70		= 230,
	NGIN3D_WORLD_71		= 231,
	NGIN3D_WORLD_72		= 232,
	NGIN3D_WORLD_73		= 233,
	NGIN3D_WORLD_74		= 234,
	NGIN3D_WORLD_75		= 235,
	NGIN3D_WORLD_76		= 236,
	NGIN3D_WORLD_77		= 237,
	NGIN3D_WORLD_78		= 238,
	NGIN3D_WORLD_79		= 239,
	NGIN3D_WORLD_80		= 240,
	NGIN3D_WORLD_81		= 241,
	NGIN3D_WORLD_82		= 242,
	NGIN3D_WORLD_83		= 243,
	NGIN3D_WORLD_84		= 244,
	NGIN3D_WORLD_85		= 245,
	NGIN3D_WORLD_86		= 246,
	NGIN3D_WORLD_87		= 247,
	NGIN3D_WORLD_88		= 248,
	NGIN3D_WORLD_89		= 249,
	NGIN3D_WORLD_90		= 250,
	NGIN3D_WORLD_91		= 251,
	NGIN3D_WORLD_92		= 252,
	NGIN3D_WORLD_93		= 253,
	NGIN3D_WORLD_94		= 254,
	NGIN3D_WORLD_95		= 255,		/* 0xFF */

	/* Numeric keypad */
	NGIN3D_KP0		= 256,
	NGIN3D_KP1		= 257,
	NGIN3D_KP2		= 258,
	NGIN3D_KP3		= 259,
	NGIN3D_KP4		= 260,
	NGIN3D_KP5		= 261,
	NGIN3D_KP6		= 262,
	NGIN3D_KP7		= 263,
	NGIN3D_KP8		= 264,
	NGIN3D_KP9		= 265,
	NGIN3D_KP_PERIOD	= 266,
	NGIN3D_KP_DIVIDE	= 267,
	NGIN3D_KP_MULTIPLY	= 268,
	NGIN3D_KP_MINUS		= 269,
	NGIN3D_KP_PLUS		= 270,
	NGIN3D_KP_ENTER		= 271,
	NGIN3D_KP_EQUALS	= 272,

	/* Arrows + Home/End pad */
	NGIN3D_UP		= 273,
	NGIN3D_DOWN		= 274,
	NGIN3D_RIGHT		= 275,
	NGIN3D_LEFT		= 276,
	NGIN3D_INSERT		= 277,
	NGIN3D_HOME		= 278,
	NGIN3D_END		= 279,
	NGIN3D_PAGEUP		= 280,
	NGIN3D_PAGEDOWN		= 281,

	/* Function keys */
	NGIN3D_F1		= 282,
	NGIN3D_F2		= 283,
	NGIN3D_F3		= 284,
	NGIN3D_F4		= 285,
	NGIN3D_F5		= 286,
	NGIN3D_F6		= 287,
	NGIN3D_F7		= 288,
	NGIN3D_F8		= 289,
	NGIN3D_F9		= 290,
	NGIN3D_F10		= 291,
	NGIN3D_F11		= 292,
	NGIN3D_F12		= 293,
	NGIN3D_F13		= 294,
	NGIN3D_F14		= 295,
	NGIN3D_F15		= 296,

	/* Key state modifier keys */
	NGIN3D_NUMLOCK		= 300,
	NGIN3D_CAPSLOCK		= 301,
	NGIN3D_SCROLLOCK	= 302,
	NGIN3D_RSHIFT		= 303,
	NGIN3D_LSHIFT		= 304,
	NGIN3D_RCTRL		= 305,
	NGIN3D_LCTRL		= 306,
	NGIN3D_RALT		= 307,
	NGIN3D_LALT		= 308,
	NGIN3D_RMETA		= 309,
	NGIN3D_LMETA		= 310,
	NGIN3D_LSUPER		= 311,		/* Left "Windows" key */
	NGIN3D_RSUPER		= 312,		/* Right "Windows" key */
	NGIN3D_MODE		= 313,		/* "Alt Gr" key */
	NGIN3D_COMPOSE		= 314,		/* Multi-key compose key */

	/* Miscellaneous function keys */
	NGIN3D_HELP		= 315,
	NGIN3D_PRINT		= 316,
	NGIN3D_SYSREQ		= 317,
	NGIN3D_BREAK		= 318,
	NGIN3D_MENU		= 319,
	NGIN3D_POWER		= 320,		/* Power Macintosh power key */
	NGIN3D_EURO		= 321,		/* Some european keyboards */
	NGIN3D_UNDO		= 322,		/* Atari keyboard has Undo */

	/* Add any other keys here */

	NGIN3D_LAST
};

#endif
