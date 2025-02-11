#ifndef OBS_HPP
#define OBS_HPP

class obs{
    public:
        float time;
        int x,y;
        obs(int a,int b,float c){
            x = a;
            y = b;
            time = c;
        }
        bool operator< (const obs& other) const {
            return time > other.time;
        };
};

#endif