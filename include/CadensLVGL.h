/*


Author: Caden Gross

Title: Cadens LVGL Extension

Version: 0.5 Beta

Description: Adds to LVGL for easier use and understanding.


Wanted Future features:
Error label thats defaultly made and configured

*/

#ifndef _CadensLVGL_H_
#define _CadensLVGL_H_

#include "liblvgl/core/lv_obj_style.h"
#include "liblvgl/core/lv_obj_tree.h"
#include "liblvgl/misc/lv_style.h"
#include "liblvgl/misc/lv_style_gen.h"
#include "liblvgl/misc/lv_types.h"
#include "liblvgl/widgets/dropdown/lv_dropdown.h"
#include "liblvgl/widgets/line/lv_line.h"
#include "main.h" // IWYU pragma: keep
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/misc/lv_color.h"
#include <cstdarg>
#include <cstring>  
#include <cstddef> // IWYU pragma: keep
#include <vector>
#include <climits>
#include <stdio.h> // IWYU pragma: keep
#include <stdarg.h>
#include <string>

/**
 * Creates a button
 *
 * Example:
 * lv_obj_t * button = createButton(LV_ALIGN_CENTER, 12, 12, 120, 120, style, screen); 
 * lv_obj_t * unneededlabel = addText(button, "Button Text");
 *
 * @param Alignment The alignment of the object
 * @param xOffset   The x position of the object, offsetted from the alignment
 * @param yOffset   The y position of the object, offsetted from the alignment
 * @param width     The width of the object
 * @param height    The height of the object
 * @param style     The objects style
 * @param screen    The screen/parent object of the child object
 * @return          The button
 */
inline lv_obj_t * createButton(lv_align_t Alignment, int xOffset, int yOffset, int width, int height, lv_style_t & style, lv_obj_t * screen = lv_screen_active()) {
	lv_obj_t * button = lv_button_create(screen);
	lv_obj_add_style(button, &style, 0); /*0 means add to the main part and default state*/
	/* Set parent-sized width, and content-sized height */
	lv_obj_set_size(button, width, height);
	/* Align to the right center with 20px offset horizontally */
	lv_obj_align(button, Alignment, xOffset, yOffset);

	return button;
}

/**
 * Creates a button
 *
 * Example:
 * lv_obj_t * button = createButton(120, 50, 120, 120, style, screen); 
 * lv_obj_t * unneededlabel = addText(button, "Button Text");
 *
 * @param xPos      The x position of the object
 * @param yPos      The y position of the object
 * @param width     The width of the object
 * @param height    The height of the object
 * @param style     The objects style
 * @param screen    The screen/parent object of the child object
 * @return          The button
 */
inline lv_obj_t * createButton(int xPos, int yPos, int width, int height, lv_style_t & style, lv_obj_t * screen = lv_screen_active()) {
	lv_obj_t * button = lv_button_create(screen);
	lv_obj_add_style(button, &style, 0); /*0 means add to the main part and default state*/
	/* Set parent-sized width, and content-sized height */
	lv_obj_set_size(button, width, height);
	/* Align to the right center with 20px offset horizontally */
	lv_obj_align(button, LV_ALIGN_DEFAULT, xPos, yPos);
	
	return button;
}

/**
 * Creates a dropdown
 *
 * Example:
 * lv_obj_t * dropdown = createDropdown(LV_ALIGN_CENTER, 12, 12, 120, 120, style, screen); 
 * lv_dropdown_set_options( // Creates the dropdowns options
 *		thedropdown,
 *		"Example\n"
 *		"SecondLine\n"
 *		"ThirdOption"
 *	);
 *
 * @param Alignment The alignment of the object
 * @param xOffset   The x position of the object, offsetted from the alignment
 * @param yOffset   The y position of the object, offsetted from the alignment
 * @param width     The width of the object
 * @param height    The height of the object
 * @param style     The objects style
 * @param screen    The screen/parent object of the child object
 * @return          The dropdown
 */
inline lv_obj_t * createDropDown(lv_align_t Alignment, int xOffset, int yOffset, int width, int height, lv_style_t & style, lv_obj_t * screen = lv_screen_active()) {
	lv_obj_t * list = lv_dropdown_create(screen);
	lv_obj_add_style(list, &style, 0); /*0 means add to the main part and default state*/
	/* Set parent-sized width, and content-sized height */
	lv_obj_set_size(list, width, height);
	/* Align to the right center with 20px offset horizontally */
	lv_obj_align(list, Alignment, xOffset, yOffset);

	return list;
}

/**
 * Creates a dropdown
 *
 * Example:
 * lv_obj_t * dropdown = createDropdown(LV_ALIGN_CENTER, 12, 12, 120, 120, style, screen); 
 * lv_dropdown_set_options( // Creates the dropdowns options
 *		thedropdown,
 *		"Example\n"
 *		"SecondLine\n"
 *		"ThirdOption"
 *	);
 *
 * @param xPos      The x position of the object
 * @param yPos      The y position of the object
 * @param width     The width of the object
 * @param height    The height of the object
 * @param style     The objects style
 * @param screen    The screen/parent object of the child object
 * @return          The dropdown
 */
inline lv_obj_t * createDropDown(int xPos, int yPos, int width, int height, lv_style_t & style, lv_obj_t * screen = lv_screen_active()) {
	lv_obj_t * list = lv_button_create(screen);
	lv_obj_add_style(list, &style, 0); /*0 means add to the main part and default state*/
	/* Set parent-sized width, and content-sized height */
	lv_obj_set_size(list, width, height);
	/* Align to the right center with 20px offset horizontally */
	lv_obj_align(list, LV_ALIGN_DEFAULT, xPos, yPos);

	return list;
}

/**
 * Creates a button
 *
 * Example:
 * lv_obj_t * tabview = createTabView(60, style, example_screen, LV_DIR_TOP);
 * 
 * lv_obj_t * tab1 = lv_tabview_add_tab(tabview, "Test1");
 * lv_obj_t * tab2 = lv_tabview_add_tab(tabview, "Test2");
 * lv_obj_t * tab3 = lv_tabview_add_tab(tabview, "Test3");
 *
 * @param width     The width of the dropdown
 * @param style     The objects style
 * @param screen    The screen/parent object of the child object
 * @param dir       The direction of the screen that the object is on
 * @return          The button
 */
inline lv_obj_t * createTabView(int width, lv_style_t & style, lv_obj_t * screen = lv_screen_active(), lv_dir_t dir = LV_DIR_TOP) {
	lv_obj_t * tabview;
    tabview = lv_tabview_create(screen);

	lv_tabview_set_tab_bar_position(tabview, dir);
    lv_tabview_set_tab_bar_size(tabview, width);
	lv_obj_add_style(tabview, &style, 0); /*0 means add to the main part and default state*/

	lv_obj_remove_flag(lv_tabview_get_content(tabview), LV_OBJ_FLAG_SCROLLABLE);

	return tabview;
}

/**
 * DOESN'T FULLY FUNCTION YET, DO NOT TRY PUTTING VARIABLES IN THESE.
 *
 * Example:
 * lv_obj_t * label = addText(button, "ButtonText");
 * 
 *
 * @param parent	The parent object
 * @param fmt       The text and stuff
 * @return          The label
 * @warning         There is a different version of addText for labels not just text on another object
*/
inline lv_obj_t * addText(lv_obj_t * parent, const char * fmt, ...) {
	lv_obj_t * label = lv_label_create(parent);

	va_list args;

	va_start(args, fmt);

	char buffer[256]; // There really shouldn't be a scenario where we add a text with more than 256 so I don't care

	lv_vsnprintf(buffer, sizeof(buffer), fmt, args);
	lv_label_set_text_fmt(label, "%s", buffer); 

	return label;
}

/** 
 * DOESN'T FULLY FUNCTION YET, DO NOT TRY PUTTING VARIABLES IN THESE.
 *
 * Example:
 * lv_obj_t * label = addText(datscreen, style, LV_ALIGN_DEFAULT, 5, 10, "Text");
 * 
 *
 * @param parent	The parent object
 * @param style     The labels style
 * @param Alignment The alignment of the object
 * @param xOffset   The x position of the object, offsetted from the alignment
 * @param yOffset   The y position of the object, offsetted from the alignment
 * @param fmt       The text and stuff
 * @return          The label
 * 
*/
inline lv_obj_t * addText(lv_obj_t * parent, lv_style_t & style, lv_align_t Alignment, int xOffset, int yOffset, const char * fmt, ...) {
	lv_obj_t * label = lv_label_create(parent);

	va_list args;
	va_start(args, fmt);
	char buffer[256]; // There really shouldn't be a scenario where we add a text with more than 256 so I don't care

	lv_vsnprintf(buffer, sizeof(buffer), fmt, args);
	lv_label_set_text_fmt(label, "%s", buffer);
	lv_obj_add_style(label, &style, 0);

	int xo = (xOffset != INT_MIN) ? xOffset : 0;
	int yo = (yOffset != INT_MIN) ? yOffset : 0;
	lv_obj_align(label, Alignment, xo, yo);

	return label;
}


/** 
 * DOESN'T FULLY FUNCTION YET, DO NOT TRY PUTTING VARIABLES IN THESE.
 *
 * Example:
 * lv_obj_t * label = addText(datscreen, style, 5, 10, "Text");
 * 
 *
 * @param parent	The parent object
 * @param style     The labels style
 * @param xPos      The x position of the object
 * @param yPos      The y position of the object
 * @param fmt       The text and stuff
 * @return          The label
 * 
*/
inline lv_obj_t * addText(lv_obj_t * parent, lv_style_t & style, int xPos, int yPos, const char * fmt, ...) {
	lv_obj_t * label = lv_label_create(parent);

	va_list args;
	va_start(args, fmt);
	char buffer[256]; // There really shouldn't be a scenario where we add a text with more than 256 so I don't care

	lv_vsnprintf(buffer, sizeof(buffer), fmt, args);
	lv_label_set_text_fmt(label, "%s", buffer);
	lv_obj_add_style(label, &style, 0);

	lv_obj_align(label, LV_ALIGN_DEFAULT, xPos, yPos);

	return label;
}


/** 
 * DOESN'T FULLY FUNCTION YET, DO NOT TRY PUTTING VARIABLES IN THESE.
 *
 * Example:
 * updateText(buttonLabel, "New Text");
 * 
 *
 * @param obj      	The label you are updating
 * @param fmt       The text and stuff
 * 
*/
inline void updateText(lv_obj_t * obj, const char * fmt, ...) {
	va_list args;

	va_start(args, fmt);

	char buffer[256]; // There really shouldn't be a scenario where we add a text with more than 256 so I don't care

	lv_vsnprintf(buffer, sizeof(buffer), fmt, args);
	lv_label_set_text_fmt(obj, "%s", buffer);
}


inline void styleUpdate(lv_style_t & style, const char * option, ...) {

	va_list args; // Creates the list of arguments 
	va_start(args, option);

	if (strcmp(option, "Background Color: ") == 0) {
        lv_style_set_bg_color(&style, lv_color_hex(va_arg(args, int)));
    }

	if (strcmp(option, "Border Width: ") == 0) {
        lv_style_set_border_width(&style, va_arg(args, int));
    }

	if (strcmp(option, "Line Color: ") == 0) {
		lv_style_set_line_color(&style, lv_color_hex(va_arg(args, int)));
    }

	if (strcmp(option, "Line Width: ") == 0) {
		lv_style_set_line_width(&style, va_arg(args, int));
    }

	if (strcmp(option, "Line Rounded: ") == 0) {
		lv_style_set_line_rounded(&style, va_arg(args, int));
    }

	if (strcmp(option, "Opacity: ") == 0) {
		lv_style_set_opa(&style, va_arg(args, int));
    }

	if (strcmp(option, "Text Alignment: ") == 0) {
		lv_style_set_text_align(&style, va_arg(args, lv_text_align_t));
    }
}

class clvgl_line {
private:
	lv_obj_t* line_obj;

	// Big permanent storage that is a 2d array which stores all the points
	std::vector<lv_point_precise_t> m_points; 

public:
	// Big magic constructor which takes the temp array given to the function and makes sure it is turn permanent
	clvgl_line(const std::vector<lv_point_precise_t>& initial_points, lv_style_t& style, lv_obj_t * screen = lv_screen_active()) { 
		// Copys the temp points into the m_points vector
		m_points = initial_points;

		line_obj = lv_line_create(screen);

		lv_line_set_points(line_obj, m_points.data(), m_points.size());
        lv_obj_add_style(line_obj, &style, 0);
	}

	// Clean up
	~clvgl_line() {
		lv_obj_delete(line_obj);
	}
};

class cterminal {
	private:
		static constexpr int TERMINAL_SIZE = 14; // Change this to adjust terminal size
		std::string lines[TERMINAL_SIZE];
		lv_obj_t * rows[TERMINAL_SIZE];

		void initTerminalRows() {
			for (int i = 0; i < TERMINAL_SIZE; i++) {
				rows[i] = lv_label_create(terminalScreen);
				lv_label_set_text_fmt(rows[i], "%s", "Init Line");
				lv_obj_align(rows[i], LV_ALIGN_DEFAULT, 5, i * 16);
			}
		}

		void printTerminalScreen() {
			for (int i = 0; i < TERMINAL_SIZE; i++) {
				std::string line = lines[i];
				lv_label_set_text_fmt(rows[i], "%s", line.c_str());
			}
		}
		

	public:

		lv_obj_t * terminalScreen = NULL;
		int currentLine = 0;
	
		// Scrolls every line by 1 to make room for newest one
		void scroll() {
			std::string linesBuffer[TERMINAL_SIZE];
			// Scroll into buffer
			for (int i = 0; i < TERMINAL_SIZE - 1; i++) {
				linesBuffer[i] = lines[i + 1];
			}
			currentLine -= 1;
			linesBuffer[TERMINAL_SIZE - 1] = ".";

			// Set lines to buffer
			for (int i = 0; i < TERMINAL_SIZE; i++) {
				lines[i] = linesBuffer[i];
			}
		}

		void clear() {
			currentLine = 0;
			for (int i = 0; i < TERMINAL_SIZE; i++) {
				lines[i] = ".";
			}
		}

		void print(std::string line) {
			currentLine++;
			if (currentLine == TERMINAL_SIZE) {
				scroll();
			}
			lines[currentLine - 1] = line;
		}
		
		void printLine(int row, std::string line) {
			lines[row - 1] = line;
		}

		void init() {
			terminalScreen = lv_obj_create(NULL);

			initTerminalRows();
			clear(); // Running this at first inits lines array

			pros::Task terminalUpdatter([&]() {
				while(true) {
        			printTerminalScreen();
					pros::delay(60); // Waits one second before resetting it again so that we don't keep it at 100
				}
			});
		}

};

#endif