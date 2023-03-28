# N64 DreamOS ROM (Menu ROM)

To build these, make sure to install libdragon first. Ensure that the environment variable `N64_INST` is correctly set, then run `make`. If you're using the dockerized version, use `libdragon make` instead.

To flash this to a Dreamdrive64, follow those instructions accordingly.

This is the menu rom for the Dreamdrive64. It could be run on other flash carts but some code will need to be modified in order for it function appropriatly. 
`loadRomAtSelection` and `ddr64_sd_wait_single` would need to be updated with the appropriate registers or access patterns for the cart being used.
