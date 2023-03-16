#pragma once

#define MAX_VISIBLE_CHARACTERS_LIST_VIEW (36) // 36 characters can fit with the current margin and spacing
#define MAX_VISIBLE_CHARACTERS_INFO_PANE (24) // 24 characters can fit with the current margin and spacing

typedef struct {
    int current_tick;       // How many frames/ticks deep we are.
    int start_delay;        // Number of frames to delay before starting animation.
    int max_ticks;          // number of frames per animation "step". Reset `current_frame` once reached.
} animation_info_t;

typedef struct {
    bool needs_to_scroll;
    int visible_start_char_index;
    int max_visible;
    //bool wrap; // If the text should not have any white space at the "end" and simply start repeating the string
    int direction; // positive = text moves right to left, negative = text moves left to right
    bool soft_scroll; // When true, the text in the visual buffer will stream back in from the appropriate direction instead of popping in all at once
    char* visible_text_buffer;
    char* original_text;
    animation_info_t animation_info;

    // Number of characters that were last visible
    int last_num_visible;
} animation_text_scroll_t;

typedef struct {
    int total_num_frames;
    int current_frame;
    // sprite_t** image_collection;
    animation_info_t animation_info;
} animation_image_t;

void init_scrolling_text_animation(animation_text_scroll_t* animation);
void update_scrolling_text_animation(animation_text_scroll_t* animation);
void update_sprite_animation(animation_image_t* animation);