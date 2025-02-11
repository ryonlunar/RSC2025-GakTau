#ifndef VICT_HPP
#define VICT_HPP

class vic{
    public:
        float time;
        int x,y;
        vic(int a,int b,float c){
            x = a;
            y = b;
            time = c;
        }
        bool operator< (const vic& other) const {
            return time > other.time;
        };
};

#endif