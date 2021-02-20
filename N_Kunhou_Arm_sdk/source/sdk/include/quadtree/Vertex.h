#ifndef _VERTEX_QUADTREE_2018_08_022_
#define _VERTEX_QUADTREE_2018_08_022_

class vertex
{
    public:

        vertex ()
        {
            x = 0;
            y = 0;
			ref_id_ = -1;
        }
		vertex (long double newX, long double newY)
		{
			x = newX;
			y = newY;
			ref_id_ = -1;
		}
        vertex (long double newX, long double newY , int ref_id)
        {
            x = newX;
            y = newY;
			ref_id_ = ref_id;
        }
        ~vertex (){}

        long double x;
        long double y;
		int ref_id_;

        bool operator == (vertex v){ return ((x == v.x)&&(y==v.y)&&(ref_id_==v.ref_id_)); }
};
#endif//_VERTEX_QUADTREE_2018_08_022_