#include <libdragon.h>
#include <string.h>
#include <stdlib.h>
#include "animation.h"

void init_scrolling_text_animation(animation_text_scroll_t* animation) {
    
    animation->visible_text_buffer = malloc(sizeof(char) * animation->max_visible);
    animation->animation_info.current_tick = 0;
    animation->visible_start_char_index = 0;
    strncpy(animation->visible_text_buffer, 
        &(animation->original_text)[animation->visible_start_char_index], 
        animation->max_visible
    );
}

void update_scrolling_text_animation(animation_text_scroll_t* animation) {
    
    // if (currently_selected != calculated_last_selected_item_index) {
    //     animation->animation_info.current_tick = 0;
    //     calculated_last_selected_item_index = currently_selected;
    //     animation->visible_start_char_index = 0;
    //     strncpy(animation->visible_text_buffer, 
    //         &(g_current_dir_entries[currently_selected]->filename)[animation->visible_start_char_index], 
    //         MAX_VISIBLE_CHARACTERS_LIST_VIEW
    //     );
    //     animation->needs_to_scroll = (strlen(g_current_dir_entries[currently_selected]->filename) > MAX_VISIBLE_CHARACTERS_LIST_VIEW);
    // }

    // Only need to calculate if there is text to scroll
    if (!animation->needs_to_scroll) {
        return;
    }

    // Scroll "normally", right to left
    if(animation->direction >= 0) {

        if (
            (animation->visible_start_char_index != 0 && animation->animation_info.current_tick > animation->animation_info.max_ticks) || 
            (animation->visible_start_char_index == 0 && animation->animation_info.current_tick > animation->animation_info.start_delay)
        ) {
            animation->animation_info.current_tick = 0;
            animation->visible_start_char_index++;

            int selection_text_length = strlen(animation->original_text);
            if (animation->visible_start_char_index >= selection_text_length) {
                animation->visible_start_char_index = 0;
            }

            // Only need to copy if the text is going to change
            int charsToCopy = animation->max_visible;
            if ((selection_text_length - animation->visible_start_char_index) > animation->max_visible) {
                charsToCopy = animation->max_visible;
            } else {
                charsToCopy = selection_text_length - animation->visible_start_char_index;
            }

            strncpy(animation->visible_text_buffer, &(animation->original_text)[animation->visible_start_char_index], charsToCopy);
            animation->visible_text_buffer[charsToCopy] = '\0';
        }

    // Scroll from left to right
    } else {
        int text_length = strlen(animation->original_text);

        if (
            (animation->visible_start_char_index != text_length && animation->animation_info.current_tick > animation->animation_info.max_ticks) || 
            (animation->visible_start_char_index == text_length && animation->animation_info.current_tick > animation->animation_info.start_delay)
        ) {
            animation->animation_info.current_tick = 0;
            animation->visible_start_char_index--;

            if (animation->visible_start_char_index <= 0) {
                animation->visible_start_char_index = text_length;
            }

            int charEnd     = animation->visible_start_char_index;
            int charStart   = charEnd - animation->max_visible;
            int paddingSpaces = 0;
            // Will need some leading spaces
            if (charStart < 0) {
                paddingSpaces = 0 - charStart;
                charStart = 0;
            }

            int charsToCopy = charEnd - charStart;
            if(animation->soft_scroll && (animation->visible_start_char_index > (strlen(animation->original_text) - animation->max_visible))) {
                charsToCopy = strlen(animation->original_text) - animation->visible_start_char_index;
            }

            if (paddingSpaces > 0) {
                charsToCopy = animation->max_visible - paddingSpaces;
                // strncpy(&animation->visible_text_buffer[paddingSpaces], &(animation->original_text)[0], charEnd);
                for(int i = 0; i < animation->max_visible; i++) {
                    if (i < paddingSpaces) {
                        animation->visible_text_buffer[i] = ' ';
                    } else {
                        animation->visible_text_buffer[i] = animation->original_text[i-paddingSpaces];
                    }
                }
            } else {
                strncpy(animation->visible_text_buffer, &(animation->original_text)[charStart], charsToCopy);
                animation->visible_text_buffer[charsToCopy] = '\0';
            }
        }

    }

    animation->animation_info.current_tick++;
}

void update_sprite_animation(animation_image_t* animation) {
    if (animation->animation_info.current_tick > animation->animation_info.max_ticks) {
        animation->animation_info.current_tick = 0;
        animation->current_frame++;
        if (animation->current_frame >= animation->total_num_frames) {
            animation->current_frame = 0;
        }
    }
    
    animation->animation_info.current_tick++;
}