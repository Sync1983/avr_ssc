/*
    RingBufferT.h - A ring buffer template class for AVR processors.
    For AVR ATMega328p (Arduino Uno) and ATMega2560 (Arduino Mega).
    This is part of the AVRTools library.
    Copyright (c) 2014 Igor Mikolic-Torreira.  All right reserved.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/




#ifndef RingBufferT_h
#define RingBufferT_h


#include <util/atomic.h>


template< typename T, typename N, unsigned int SIZE > class RingBufferT
{

public:

    RingBufferT()
        : mSize( SIZE ), mLength( 0 ), mIndex( 0 )
        {}


    T pull()
    {
        T element;// = (0;
        ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
        {
            if ( mLength )
            {
                element = mBuffer[ mIndex ];
                mIndex++;
                if ( mIndex >= mSize )
                {
                    mIndex -= mSize;
                }
                --mLength;
            }
        }
        return element;
    }


    T peek( N index = 0 )
    {
        T element;
        ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
        {
            element = mBuffer[ ( mIndex + index ) % mSize ];
        }
        return element;
    }


    bool push( T element )
    {
        ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
        {
            if ( mLength < mSize )
            {
                mBuffer[ ( mIndex + mLength ) % mSize ] = element;
                ++mLength;
                return 0;
            }
        }
        // True = failure
        return 1;
    }


    bool isEmpty()
    {
        return !static_cast<bool>( mLength );
    }


    bool isNotEmpty()
    {
        return static_cast<bool>( mLength );
    }


    bool isFull()
    {
        ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
        {
            return ( mSize - mLength ) <= 0;
        }
    }


    bool isNotFull()
    {
        ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
        {
            return ( mSize - mLength ) > 0;
        }
    }


    void discardFromFront( N nbrElements )
    {
        ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
        {
            if ( nbrElements < mLength )
            {
                mIndex += nbrElements;
                if( mIndex >= mSize )
                {
                    mIndex -= mSize;
                }
                mLength -= nbrElements;
            }
            else
            {
                // flush the whole buffer
                mLength = 0;
            }
        }
    }


    void clear()
    {
        ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
        {
            mLength = 0;
        }
    }



private:

    T mBuffer[ SIZE ] ;
    volatile N mSize;
    volatile N mLength;
    volatile N mIndex;

};


#endif