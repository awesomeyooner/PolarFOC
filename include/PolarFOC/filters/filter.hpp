#ifndef FILTER_HPP
#define FILTER_HPP


class Filter
{

    public:

        virtual void update(double incoming_data) = 0;

        virtual double get() = 0;
        
        virtual double get(double incoming_data);

    private:



}; // class Filter


#endif // FILTER_HPP