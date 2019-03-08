

void gray8_to_RGB565_crop(unsigned char *input,unsigned short *output,int w, int h,int nw, int nh)
{
  int a=0;
  for(int i=2;i<h-2;i++)
  {
    for(int j=2;j<w-2;j++)
    {
      output[a++]=((input[i*w+j] >> 3 ) << 3) | ((input[i*w+j] >> 5) ) | (((input[i*w+j] >> 2 ) << 13) )|   ((input[i*w+j] >> 3) <<8);
    }
  }

}


void gray8_to_RGB565(unsigned char *input,unsigned short *output,int width, int height)
{

    for(int i=0;i<width*height;i++)
    {
        //output[i] = ((input[i] >> 3 ) << 11) | ((input[i] >> 2) << 5) | (input[i] >> 3);
        //output[i] = output[i] << 8 |  output[i] >> 8;
        output[i] = ((input[i] >> 3 ) << 3) | ((input[i] >> 5) ) | (((input[i] >> 2 ) << 13) )|   ((input[i] >> 3) <<8);
    }

}

