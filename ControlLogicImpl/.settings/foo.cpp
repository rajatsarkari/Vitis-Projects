void foo(int in_r[3], char a, char b, char c, int out_r[3]){
	int x, y;
	for (int i = 0; i < 3; i++){
		x = in_r[i];
		y = x*a+b+c;
		out_r[i] = y;
	}
}
