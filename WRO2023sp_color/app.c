void obj_know(int num){
    switch(obj){
        case 2:
            location[num] = 2;
            break;
        case 3:
            location[num] = 3;
            break;
        case 0:
        case 1:
        case 4:
        case 5:
        case 6:
            if(h < 150) location[num] = 3;
            else location[num] = 2;
            break;
        default:
            break;
    } 
    if(num <= 1)location_t[num] = location[num];
    else location_f[num - 2] = location[num];
    fprintf(bt, "RESULT = T=%d,%d F=%d,%d,%d,%d\r\n-----------------\r\n", location_t[0], location_t[1], location_f[0], location_f[1], location_f[2], location_f[3]);
    file=fopen(logfilename,"a");//ファイルをオープン(名前の指定)
    fprintf(file, "RESULT = T=%d,%d F=%d,%d,%d,%d\r\n-----------------\r\n", location_t[0], location_t[1], location_f[0], location_f[1], location_f[2], location_f[3]);
    fclose(file);
}

B&Gの場所でのカラーの判別方法。if文は救済措置。BLACKの見え方だけ要注意（青と色が似ていて読みづらいかもだから）

//RED
    case 5:
    case 7:
        location[num] = 5;
        break;
    if(r / b > 3) location[num] = 5;

//YELLOW
    case 4:
    case 7:
        location[num] = 4;
        break;
    if(r / b > 3) location[num] = 4;

//BLACK
    case 0:
    case 1:
        if(judge < 30 || v < 6) location[num] = 0;
        break;


//WHITE
    case 6:
        location[num] = 6;
        break;
    if(judge > 200 || max / min < 2) location[num] = 6;


WHITEの場所でのカラーの判別方法。if文は救済措置。BLACKの見え方だけ変更あり。judgeだけで読めると思います!!!

//BLACK
    case 0:
    case 1:
        if(judge < 70 || v < 6) location[num] = 0;
        break;

