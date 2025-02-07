
#ifndef MEMORY_BANK_HPP
#define MEMORY_BANK_HPP
#pragma once

#include <deque>
#include <memory>

struct Quad;

class MemoryBank {
public:
    MemoryBank();
    ~MemoryBank();
    MemoryBank( const MemoryBank& );
    MemoryBank( MemoryBank&& );

    std::unique_ptr< Quad > get();
    void store( std::unique_ptr< Quad > ToAdd );

private:
    void allocate();

    std::deque< std::unique_ptr< Quad > > Bank;

    const size_t BlockSize = 512;

    size_t TotalSize = 0;
    size_t UsedSize = 0;
};

#endif
