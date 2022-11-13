
colorid_t sp_color;


void obj_check_final(int num, way_t sensor){
    switch (location[num]) {
    case NOTHING:
        break;
    case CHEMICAL:
        break;
    case FIRE:
    case PERSON:
    default:
        break;
    }
}

void left_check(int num, way_t sensor){
    switch (obj) {
    case 0:
        sp_color = COLOR_NOTHING;
        break;
    case 1:
        sp_color = COLOR_BLUE;
        break:
    case 2:
        sp_color = COLOR_BLUE;
        break;
    case 3:
        sp_color = COLOR_GREEN;
        break;
    case 4:
        sp_color = COLOR_GREEN;
        break;
    case 5:
        sp_color = COLOR_RED;
        break;
    case 6:
        sp_color = COLOR_RED;
        break;
    case 7:
        sp_color = COLOR_RED;
        break;
    case 8:
        sp_color = COLOR_RED;
        break;
    case 9:
        sp_color = COLOR_RED;
        break;
    case 10:
        break;
    case 11:
        break;
    

            
    
    default:
        break;
    }
}