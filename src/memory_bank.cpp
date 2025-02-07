
#include <fmt/core.h>

#include "memory_bank.hpp"
#include "trace.hpp"

#include "quadtree.hpp"

MemoryBank::MemoryBank() { allocate(); }

MemoryBank::~MemoryBank() {
    if ( UsedSize > 0 ) {
        Trace::message( fmt::format( "Missing memory blocks: {}", UsedSize ) );
    }
}

MemoryBank::MemoryBank( const MemoryBank& ) {}

MemoryBank::MemoryBank( MemoryBank&& ) {}

std::unique_ptr< Quad > MemoryBank::get() {
    UsedSize += 1;

    if ( Bank.empty() ) allocate();

    std::unique_ptr< Quad > Front = std::move( Bank.front() );
    Bank.pop_front();

    Front->init();
    return Front;
}
void MemoryBank::store( std::unique_ptr< Quad > ToAdd ) {
    Bank.push_back( std::move( ToAdd ) );
    UsedSize -= 1;
}

void MemoryBank::allocate() {
    for ( size_t i = 0; i < BlockSize; ++i ) {
        Bank.push_back( std::make_unique< Quad >() );
        TotalSize += 1;
    }
}
