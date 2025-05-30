#include<iostream>
namespace rt_reachability{
    class Grid{
        public:
            Grid(int Nx, int Ny, int Nv, int Nt){
                this->Nx = Nx;
                this->Ny = Ny;
                this->Nv = Nv;
                this->Nt = Nt;
                this->value_function = (float****)malloc(Nx*sizeof(float***));
                for(int i = 0;i < Nx;i++){
                    value_function[i] = (float***)malloc(Ny*sizeof(float**));
                    for(int j = 0; j < Ny;j++){
                        value_function[i][j] = (float**)malloc(Nv*sizeof(float*));
                        for(int k = 0;k < Nv;k++){
                            value_function[i][j][k] = (float*)malloc(Nt*sizeof(float));
                            for(int l = 0;l < Nt;l++){
                                value_function[i][j][k][l] = 0.0;
                            }
                        }
                    }
                }
            }
            void setBounds(float min_x, float max_x, float min_y, float max_y, float min_v, float max_v, float min_t, float max_t){
                this->max_x = max_x;
                this->max_y = max_y;
                this->max_v = max_v;
                this->max_t = max_t;
                this->min_x = min_x;
                this->min_y = min_y;
                this->min_v = min_v;
                this->min_t = min_t;
            }
            float getValue(float x, float y, float v, float theta){
                
            }
        private:
            int Nx, Ny, Nv, Nt;
            float min_x, max_x, min_y, max_y, min_v, max_v, min_t, max_t;
            float**** value_function;
            
    };
}