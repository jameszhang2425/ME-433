char message[50];
int i = 0;
sprint(message, "hello %d", i);
draw_char(64, 12, message)

void draw_char(int x, int y, char letter){
    for (int ii = 0; i < 5; i++){
        char c = ASCII[letter-32][ii];
        for (int jj = 0; jj < 8; jj++){
            char bit = (c>>jj)&0b1;
            if (bit == 0){
                drawPixel(x+ii, y+jj, 0)
            } else{
                drawPixel(x+ii, y+jj, 1)
            }
        }
    }
}

void draw_message(int x, int y, char * m){
    kk = 0;
    while(m[kk]){
        draw_char(x+kk*5, y, m[kk]);
        kk++;
    }
}