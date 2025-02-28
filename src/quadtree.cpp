
#include <limits>

#include <fmt/core.h>

#include "quadtree.hpp"
#include "trace.hpp"

void Quad::init() {
    Children = 0;
    Next = 0;
    Size = 0.f;
    HalfSize = 0.f;
    BodyId = -1;
    Body = nullptr;
}

unsigned Quad::findQuad( const Vector2& Pos ) {
    return ( static_cast< unsigned >( Pos.y > Center.y ) << 1 |
             static_cast< unsigned >( Pos.x > Center.x ) );
}

bool Quad::intersects( const Vector2& Pos, const float HalfSize_ ) const {
    bool NotIntersects = ( Pos.x - HalfSize_ > Center.x + HalfSize ) ||
                         ( Pos.x + HalfSize_ < Center.x - HalfSize ) ||
                         ( Pos.y - HalfSize_ > Center.y + HalfSize ) ||
                         ( Pos.y + HalfSize_ < Center.y - HalfSize );

    return !NotIntersects;
}

bool Quad::hasChildren() const { return Children != 0; }

bool Quad::isEmpty() const { return BodyId == -1; }

void Quad::subdivide( Quad* Parent, unsigned QuadrantId ) {
    Size = Parent->Size * 0.5f;
    HalfSize = Size * 0.5f;
    Center.x = Parent->Center.x +
               ( static_cast< float >( QuadrantId & 1 ) - 0.5f ) * Size;
    Center.y = Parent->Center.y +
               ( static_cast< float >( QuadrantId >> 1 ) - 0.5f ) * Size;
}

void Quad::printSimple( const unsigned Id ) const {
    Trace::message( fmt::format(
        "ID: {:>4}, Children: {:>4}, Next: {:>4}, Size: {:>4}, BodyId: {:>4} ",
        Id, Children, Next, Size, BodyId ) );
}

Quadtree::Quadtree() : Mb( std::make_unique< MemoryBank >() ) {
    Theta = 0.20f;
    SquareTheta = Theta * Theta;
}

Quadtree::~Quadtree() {
    for ( auto& Node : Nodes ) {
        Mb->store( std::move( Node ) );
    }
}

Quadtree::Quadtree( const Quadtree& ) {}

Quadtree::Quadtree( Quadtree&& ) {}

std::vector< Boid* > Quadtree::query( const Vector2& Pos,
                                      const float HalfSize ) {
    std::vector< Boid* > Targets;

    query( Targets, Nodes[Root].get(), Pos, HalfSize );

    return Targets;
}

void Quadtree::query( std::vector< Boid* >& Targets, const Quad* Node,
                      const Vector2& Pos, const float HalfSize ) {
    if ( Node->intersects( Pos, HalfSize ) ) {
        if ( Node->Body != nullptr ) Targets.push_back( Node->Body );

        if ( Node->hasChildren() ) {
            for ( unsigned i = Node->Children; i < Node->Children + 4; ++i ) {
                query( Targets, Nodes[i].get(), Pos, HalfSize );
            }
        }
    }
}

void Quadtree::insert( Boid* ThisBody ) {
    unsigned NodeId = 0;

    // Finding the smallest quadrant without children
    while ( Nodes[NodeId]->hasChildren() ) {
        unsigned QuadrantId =
            Nodes[NodeId]->findQuad( ThisBody->getPosition() );

        NodeId = Nodes[NodeId]->Children + QuadrantId;
    }

    auto& CurrentNode = Nodes[NodeId];

    // If Quadrant is empty insert ThisBody and return
    if ( CurrentNode->isEmpty() ) {
        CurrentNode->BodyId = static_cast< int >( ThisBody->getId() );
        CurrentNode->Body = ThisBody;
        return;
    }

    // Else if Quadrant is not empty

    Boid* OtherBody = Nodes[NodeId]->Body;
    const int Id = Nodes[NodeId]->BodyId;

    // If two bodies are in the same location
    if ( Vector2Equals( OtherBody->getPosition(), ThisBody->getPosition() ) ) {
        Trace::message( "In same location." );
        return;
    }

    Nodes[NodeId]->Body = nullptr;
    Nodes[NodeId]->BodyId = -1;

    // Break current Quadrant into four smaller quadrants
    while ( true ) {
        unsigned ChildrenId = subdivide( NodeId );

        unsigned Q1 = Nodes[NodeId]->findQuad( OtherBody->getPosition() );
        unsigned Q2 = Nodes[NodeId]->findQuad( ThisBody->getPosition() );

        if ( Q1 == Q2 )
            NodeId = ChildrenId + Q1;
        else {
            unsigned N1 = ChildrenId + Q1;
            unsigned N2 = ChildrenId + Q2;

            Nodes[N1]->BodyId = Id;
            Nodes[N1]->Body = OtherBody;

            Nodes[N2]->BodyId = static_cast< int >( ThisBody->getId() );
            Nodes[N2]->Body = ThisBody;

            return;
        }
    }
}

unsigned Quadtree::subdivide( unsigned NodeId ) {
    Parents.push_back( NodeId );
    unsigned ChildrenId = static_cast< unsigned >( Nodes.size() );
    Nodes[NodeId]->Children = ChildrenId;

    for ( unsigned i = 1; i < 5; ++i ) {
        Nodes.push_back( std::move( Mb->get() ) );
        Nodes.back()->subdivide( Nodes[NodeId].get(), i - 1 );
        if ( i == 4 )
            Nodes.back()->Next = Nodes[NodeId]->Next;
        else
            Nodes.back()->Next = ChildrenId + i;
    }

    return ChildrenId;
}

void Quadtree::clear() {
    for ( auto& Node : Nodes ) {
        Mb->store( std::move( Node ) );
    }
    Nodes.clear();

    Parents.clear();
}

const std::vector< std::unique_ptr< Quad > >& Quadtree::getNodes() {
    return Nodes;
}
