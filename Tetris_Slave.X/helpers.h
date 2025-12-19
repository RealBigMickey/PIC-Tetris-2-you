#pragma once
typedef enum { I, O, T, S, Z, J, L } block_type;
typedef enum { dir_left, dir_right } direction;
#define GAME_AREA_WIDTH 8
#define GAME_AREA_HEIGHT 16

#define BASE_DELAY_MS 10

static void update_score(void);
static void set_current_block(bool b[][4]);
static void rotate_current_block(direction d);
static bool current_block_is_in_illegal_position(void);
static void solidify_current_block(void);
static void check_for_full_rows(void);
static void reset_game(void);
static void drop_block_from_sky(void);
static void game_step(void);
static void check_inputs(void);
void compose_frame(void);

int game_mode = 0;

bool display_matrix[GAME_AREA_HEIGHT][GAME_AREA_WIDTH];
bool board[GAME_AREA_HEIGHT][GAME_AREA_WIDTH];


bool current_block[4][4];
block_type current_block_type;

uint8_t game_speed_ticks = 14;


block_type held_block_type;
bool has_held_block = false;
bool hold_used_this_turn = false;

int block_location_x = 0, block_location_y = 0;
int score = 0;

int gap_position = 0;


static bool block_I[4][4] = { {0,0,0,0}, {1,1,1,1}, {0,0,0,0}, {0,0,0,0} };
static bool block_O[4][4] = { {0,1,1,0}, {0,1,1,0}, {0,0,0,0}, {0,0,0,0} };
static bool block_T[4][4] = { {0,1,0,0}, {1,1,1,0}, {0,0,0,0}, {0,0,0,0} };
static bool block_S[4][4] = { {0,1,1,0}, {1,1,0,0}, {0,0,0,0}, {0,0,0,0} };
static bool block_Z[4][4] = { {1,1,0,0}, {0,1,1,0}, {0,0,0,0}, {0,0,0,0} };
static bool block_J[4][4] = { {1,0,0,0}, {1,1,1,0}, {0,0,0,0}, {0,0,0,0} };
static bool block_L[4][4] = { {0,0,1,0}, {1,1,1,0}, {0,0,0,0}, {0,0,0,0} };




static void update_score(void) {
    int x = score;
    int n = 0;
    for (unsigned i = 1 << 7; i > 0; i >>= 1) {
        if (x >= i) { x -= i; board[0][n] = true; } 
        else { board[0][n] = false; }
        n++;
    }
}

static void set_current_block(bool b[][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            current_block[i][j] = b[i][j];
        }
    }
}

static void rotate_current_block(direction d) {
    if (current_block_type != O) {
        // Transpose
        for (int i = 0; i < 3; i++) {
            for (int j = i + 1; j < 4; j++) {
                bool temp = current_block[i][j];
                current_block[i][j] = current_block[j][i];
                current_block[j][i] = temp;
            }
        }
        // Swap columns
        for (int i = 0; i < 4; i++) {
            int j = 0;
            int k = (current_block_type == I) ? 3 : 2;
            while (j < k) {
                if (d == dir_right) {
                    bool temp = current_block[i][j];
                    current_block[i][j] = current_block[i][k];
                    current_block[i][k] = temp;
                } else {
                    bool temp = current_block[j][i];
                    current_block[j][i] = current_block[k][i];
                    current_block[k][i] = temp;
                }
                j++; k--;
            }
        }
    }
}

static bool current_block_is_in_illegal_position(void) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (current_block[i][j]) {
                int x = block_location_x + j;
                int y = block_location_y + i;
                if (x < 0 || x >= GAME_AREA_WIDTH || y < 2 || y >= GAME_AREA_HEIGHT) return true;
                if (board[y][x]) return true;
            }
        }
    }
    return false;
}

static void solidify_current_block(void) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (current_block[i][j]) {
                int x = block_location_x + j;
                int y = block_location_y + i;
                if (x >= 0 && x < GAME_AREA_WIDTH && y >= 0 && y < GAME_AREA_HEIGHT) {
                    board[y][x] = true;
                }
            }
        }
    }
}

static void check_for_full_rows(void) {
    int cleared = 0;
    for (int i = 2; i < GAME_AREA_HEIGHT; i++) {
        bool row_is_full = true;
        for (int j = 0; j < GAME_AREA_WIDTH; j++) {
            if (!board[i][j]) { row_is_full = false; break; }
        }
        if (row_is_full) {
            int n = i;
            while (n - 1 >= 2) {
                for (int j = 0; j < GAME_AREA_WIDTH; j++) board[n][j] = board[n - 1][j];
                n--;
            }
            for (int j = 0; j < GAME_AREA_WIDTH; j++) board[2][j] = false;
            score++;
            update_score();
            cleared++;
            if (score % 12 == 0 && game_speed_ticks > 4) game_speed_ticks -= 2;
        }
    }
    serialPrintf("%d", cleared);
}

static void reset_game(void) {
    score = 0;
    game_speed_ticks = 16;
    for (int i = 0; i < GAME_AREA_HEIGHT; i++) {
        for (int j = 0; j < GAME_AREA_WIDTH; j++) {
            board[i][j] = false;
        }
    }
    update_score();
}



static int bag[7] = {0, 1, 2, 3, 4, 5, 6};
static int bag_index = 7;

static void drop_block_from_sky(void) {
    hold_used_this_turn = false;
    if (bag_index >= 7) {
        for (int i = 0; i < 7; i++)
            bag[i] = i;


        // Fisher-Yates Shuffle
        for (int i = 6; i > 0; i--) {
            int j = rand() % (i + 1);
            int temp = bag[i];
            bag[i] = bag[j];
            bag[j] = temp;
        }
        
        bag_index = 0; // Reset index to start of bag
    }

    int r = bag[bag_index++];
    if(r==0) { set_current_block(block_I); current_block_type=I; }
    else if(r==1) { set_current_block(block_O); current_block_type=O; }
    else if(r==2) { set_current_block(block_T); current_block_type=T; }
    else if(r==3) { set_current_block(block_S); current_block_type=S; }
    else if(r==4) { set_current_block(block_Z); current_block_type=Z; }
    else if(r==5) { set_current_block(block_J); current_block_type=J; }
    else { set_current_block(block_L); current_block_type=L; }

    block_location_x = 2;
    block_location_y = 2;
}


static void game_step(void) {
    block_location_y++;
    
    if (current_block_is_in_illegal_position()) {
        block_location_y--;
        solidify_current_block();
        check_for_full_rows();
        
        // Spawn next block
        drop_block_from_sky();
        
        // Check if the NEW block immediately collides (Game Over)
        if (current_block_is_in_illegal_position()) {
            game_mode = 2; // Switch to Game Over Mode
            compose_frame();
            serialPrint("A");
        }
    }
}

void generate_garbage(int line_count) {
    memmove(&board[0][0],
            &board[1][0],
            sizeof(board[0]) * (GAME_AREA_HEIGHT - 2));

    const int bottom_row_index = GAME_AREA_HEIGHT - 1;

    memset(&board[bottom_row_index][0], true, sizeof(board[0]));
    board[bottom_row_index][gap_position] = false;

    if (current_block_is_in_illegal_position()) {
        block_location_y--;
        solidify_current_block();
        check_for_full_rows();
        drop_block_from_sky();
        if (current_block_is_in_illegal_position()) {
            reset_game();
            drop_block_from_sky();
        }
    }
}


static void load_block_from_type(block_type t) {
    current_block_type = t;
    switch (t) {
        case I: set_current_block(block_I); break;
        case O: set_current_block(block_O); break;
        case T: set_current_block(block_T); break;
        case S: set_current_block(block_S); break;
        case Z: set_current_block(block_Z); break;
        case J: set_current_block(block_J); break;
        case L: set_current_block(block_L); break;
    }
}



static void attempt_hold_block(void) {
    if (hold_used_this_turn)
        return;

    hold_used_this_turn = true;
    block_type type_to_hold = current_block_type;

    if (!has_held_block) {
        held_block_type = type_to_hold;
        has_held_block = true;
        
        drop_block_from_sky(); 
        hold_used_this_turn = true; 
    } 
    else {
        block_type type_to_spawn = held_block_type;
        
        held_block_type = type_to_hold;
        
        load_block_from_type(type_to_spawn);
        
        block_location_x = 2;
        block_location_y = 2;
    }
}



void compose_frame(void) {
    for (int r = 0; r < GAME_AREA_HEIGHT; r++) {
        for (int c = 0; c < GAME_AREA_WIDTH; c++) {
            display_matrix[r][c] = board[r][c];
        }
    }

    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            if (current_block[r][c]) { 
                int dy = block_location_y + r;
                int dx = block_location_x + c;

                if (dy >= 0 && dy < GAME_AREA_HEIGHT && dx >= 0 && dx < GAME_AREA_WIDTH) {
                    display_matrix[dy][dx] = true;
                }
            }
        }
    }
}