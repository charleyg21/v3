Code hidden_menu_code[8];

hidden_menu_code[0] = {
    .match = Input::UP,
    .next = &hidden_menu_code[1],
    .fail = &hidden_menu_code[0],
};

hidden_menu_code[1] = {
    .match = Input::UP,
    .next = &hidden_menu_code[2],
    .fail = &hidden_menu_code[0],
};

hidden_menu_code[2] = {
    .match = Input::DOWN,
    .next = &hidden_menu_code[3],
    .fail = &hidden_menu_code[2],
};

hidden_menu_code[3] = {
    .match = Input::DOWN,
    .next = &hidden_menu_code[4],
    .fail = &hidden_menu_code[1],
};

hidden_menu_code[4] = {
    .match = Input::UP,
    .next = &hidden_menu_code[5],
    .fail = &hidden_menu_code[0],
};

hidden_menu_code[5] = {
    .match = Input::DOWN,
    .next = &hidden_menu_code[6],
    .fail = &hidden_menu_code[2],
};

hidden_menu_code[6] = {
    .match = Input::UP,
    .next = &hidden_menu_code[7],
    .fail = &hidden_menu_code[0],
};

hidden_menu_code[7] = {
    .match = Input::DOWN,
    .next = nullptr,
    .fail = &hidden_menu_code[2]
};