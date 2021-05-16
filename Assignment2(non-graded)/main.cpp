#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
 
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../stb-master/stb_image_write.h"
 
#define STB_IMAGE_IMPLEMENTATION
#include "../stb-master/stb_image.h"

#include "vector.cpp"
#include <cmath>
#include <stdio.h>
#include <iostream>

class MyImage {
    public:
        unsigned char* image;
        int W;
        int H;
        double* C;
        MyImage(unsigned char* image){
            W = 512;
            H = 512;
            int channels=3;
            int c =0;
            C = (double*) malloc(W*H*sizeof(double));
            this->image = image; 
            std::cout << "i+j" << std::endl;
            for (int i=0; i<H; ++i){
                for (int j=0; j<W; ++j){
                    // std::cout << i+j << std::endl;
                    C[i*W + j] = -1.;
                }
            }

        }
        void resetC(){
            for (int i=0; i<H; ++i){
                for (int j=0; j<W; ++j){
                    // std::cout << i+j << std::endl;
                    C[i*W + j] = -1.;
                }
            }

        }
        double intensity(int x, int y){
            if (x>=W || x<0 || y >=H || y<0){
                return 0;
            }
            return image[y*W*3 + x*3 + 0] + image[y*W*3 + x*3 + 1] + image[y*W*3 + x*3 + 2];

        };

        double energyComputer (int x, int y){
            if (x>=W || x<0 || y >=H || y<0){
                return 0;
            }
            return std::abs(intensity(x+1,y) - intensity(x-1,y)) + std::abs(intensity(x,y+1) - intensity(x,y-1));
        };

        double getCValue(int x, int y){
            if (x>=W || x<0 || y >=H || y<0){
                return INFINITY;
            } 
            else if (C[y*W + x] != -1.){
                return C[y*W + x] ;
            } 
            else if (y==0){
                C[y*W + x] =  energyComputer(x,y);
                return C[y*W + x];
            }
            else {
                double min = std::min(getCValue(x-1,y-1),std::min(getCValue(x,y-1),getCValue(x+1,y-1)));
                C[y*W + x] = min + energyComputer(x,y);
                return C[y*W + x];
            }
        };

        int getStartSeam(){
            int res = 0;
            for (int i =0 ; i<W; ++i){
                if (getCValue(i,H-1) < getCValue(res,H-1)){
                    res = i;
                }
            }
            return res;
        };

        int* constructSeam(){
            int* res = (int*) malloc(H*sizeof(int));
            res[H-1] = getStartSeam();
            int cur = res[H-1];
            double v1,v2,v3,min;
            for (int j=H-2; j >=0; --j){
                v1 = getCValue(cur,j);
                v2 = getCValue(cur-1,j);
                v3 = getCValue(cur+1,j);
                min = std::min(v1,std::min(v2,v3));
                if (v3 == min){
                    cur++; 
                }
                else if (v2 == min){
                    cur--;
                }
                
                res[j] = cur;

            }
            return res;
        };

        void removeSeam(){
            int* seam = constructSeam();
            int x;
            for (int j = 0; j< H;++j){
                x = seam[j];
                // for (int i=x ; i <W; ++i){
                    
                //         image[j*W + i*3 + 0] = image[j*W + (i+1)*3 + 0];
                //         image[j*W + i*3 + 1] = image[j*W + (i+1)*3 + 1];
                //         image[j*W + i*3 + 2] = image[j*W + (i+1)*3 + 2];
                    
                // }
                image[j*W + x*3 + 0] = 0;
                image[j*W + x*3 + 1] = 0;
                image[j*W + x*3 + 2] =0;
            }
            resetC();
            // W--;
        }

        void reduceImage(int num){
            for (int i=0; i<=num; ++i){
                removeSeam();
            }
            
        }





};





int main(){
    const char* imageFileName = "C:/Users/USUARIO/Desktop/Academics/CSE306/Tutorial5/inputimage.jpeg";
    int W_;
    int H_;
    int c =0;
    unsigned char* image= stbi_load(imageFileName, &H_, &W_, &c,3);
    MyImage imageObj(image);
    imageObj.reduceImage(1);

    // double d = imageObj.getCValue(0,0);
    // int* col = imageObj.constructSeam();
    // for (int k = imageObj.H-1; k>=0; --k){
    //     printf("(%d,%d)\n",col[k],k);
    // }
    
    stbi_write_png("image.png", 512, 512, 3, imageObj.image, 0);

    return 0;
};