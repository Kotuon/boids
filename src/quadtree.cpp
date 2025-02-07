
#include <limits>

#include <fmt/core.h>

#include "quadtree.hpp"
#include "trace.hpp"

void Quad::init() {
    Children = 0;
    Next = 0;
    Size = 0.f;
    Mass = 0.f;
}

unsigned Quad::findQuad( const Vector2& Pos ) {
    return ( static_cast< unsigned >( Pos.y > Center.y ) << 1 |
             static_cast< unsigned >( Pos.x > Center.x ) );
}

bool Quad::hasChildren() const { return Children != 0; }

bool Quad::isEmpty() const { return Mass == 0.f; }

void Quad::subdivide( Quad* Parent, unsigned QuadrantId ) {
    Size = Parent->Size * 0.5f;
    Center.x = Parent->Center.x +
               ( static_cast< float >( QuadrantId & 1 ) - 0.5f ) * Size;
    Center.y = Parent->Center.y +
               ( static_cast< float >( QuadrantId >> 1 ) - 0.5f ) * Size;
}

float Quad::getMass( const std::vector< std::unique_ptr< Quad > >& Nodes ) {
    if ( Children != 0 ) {
        Mass = Nodes[Children]->getMass( Nodes ) +
               Nodes[Children + 1]->getMass( Nodes ) +
               Nodes[Children + 2]->getMass( Nodes ) +
               Nodes[Children + 3]->getMass( Nodes );
    }

    return Mass;
}

Vector2
Quad::getCenterOfMass( const std::vector< std::unique_ptr< Quad > >& Nodes ) {
    if ( Children != 0 ) {
        CenterOfMass = Vector2Add(
            CenterOfMass,
            Vector2Add(
                Nodes[Children]->CenterOfMass,
                Vector2Add(
                    Nodes[Children + 1]->CenterOfMass,
                    Vector2Add( Nodes[Children + 2]->CenterOfMass,
                                Nodes[Children + 3]->CenterOfMass ) ) ) );
        CenterOfMass = Vector2Scale( CenterOfMass, 1.f / Mass );
    }

    return CenterOfMass;
}

void Quad::print( const unsigned Id ) const {
    Trace::message( fmt::format(
        "ID: {:>4}, Children: {:>4}, Next: {:>4}, Center: ({:>10}, {:>10}), "
        "CenterOfMass: ({:>10}, {:>10}), Size: {:>4}, Mass: {:>4} ",
        Id, Children, Next, Center.x, Center.y, CenterOfMass.x, CenterOfMass.y,
        Size, Mass ) );
}

Quadtree::Quadtree() : Mb( std::make_unique< MemoryBank >() ) {
    Theta = 0.20f;
    SquareTheta = Theta * Theta;
}

Quadtree::~Quadtree() {}

Quadtree::Quadtree( const Quadtree& ) {}

Quadtree::Quadtree( Quadtree&& ) {}

void Quadtree::insert( Boid* ThisBody ) {
    unsigned NodeId = 0;

    while ( Nodes[NodeId]->hasChildren() ) {
        unsigned QuadrantId =
            Nodes[NodeId]->findQuad( ThisBody->getPosition() );
        NodeId = Nodes[NodeId]->Children + QuadrantId;
    }

    auto& CurrentNode = Nodes[NodeId];

    if ( CurrentNode->isEmpty() ) {
        CurrentNode->CenterOfMass = ThisBody->getPosition();
        CurrentNode->Mass = 1.f;
        return;
    }

    const Vector2& Pos = Nodes[NodeId]->CenterOfMass;
    const float Mass = Nodes[NodeId]->Mass;

    if ( Vector2Equals( Pos, ThisBody->getPosition() ) ) {
        Nodes[NodeId]->Mass += 1.f;
        return;
    }

    while ( true ) {
        unsigned ChildrenId = subdivide( NodeId );

        unsigned Q1 = Nodes[NodeId]->findQuad( Pos );
        unsigned Q2 = Nodes[NodeId]->findQuad( ThisBody->getPosition() );

        if ( Q1 == Q2 )
            NodeId = ChildrenId + Q1;
        else {
            unsigned N1 = ChildrenId + Q1;
            unsigned N2 = ChildrenId + Q2;

            Nodes[N1]->CenterOfMass = Pos;
            Nodes[N1]->Mass = Mass;
            Nodes[N2]->CenterOfMass = ThisBody->getPosition();
            Nodes[N2]->Mass = 1.f;

            return;
        }
    }
}

unsigned Quadtree::subdivide( unsigned NodeId ) {
    Parents.push_back( NodeId );
    unsigned ChildrenId = static_cast< unsigned >( Nodes.size() );
    Nodes[NodeId]->Children = ChildrenId;

    for ( unsigned i = 1; i < 5; ++i ) {
        // Nodes.push_back( std::make_unique< Quad >() );
        Nodes.push_back( std::move( Mb->get() ) );
        Nodes.back()->subdivide( Nodes[NodeId].get(), i - 1 );
        if ( i == 4 )
            Nodes.back()->Next = Nodes[NodeId]->Next;
        else
            Nodes.back()->Next = ChildrenId + i;
    }

    return ChildrenId;
}

void Quadtree::propagate() {
    for ( int i = static_cast< int >( Parents.size() ) - 1; i >= 0; --i ) {
        size_t Children = Nodes[Parents[i]]->Children;

        Nodes[Parents[i]]->CenterOfMass = Vector2Add(
            Vector2Scale( Nodes[Children]->CenterOfMass,
                          Nodes[Children]->Mass ),
            Vector2Add(
                Vector2Scale( Nodes[Children + 1]->CenterOfMass,
                              Nodes[Children + 1]->Mass ),
                Vector2Add( Vector2Scale( Nodes[Children + 2]->CenterOfMass,
                                          Nodes[Children + 2]->Mass ),
                            Vector2Scale( Nodes[Children + 3]->CenterOfMass,
                                          Nodes[Children + 3]->Mass ) ) ) );

        Nodes[Parents[i]]->Mass =
            Nodes[Children]->Mass + Nodes[Children + 1]->Mass +
            Nodes[Children + 2]->Mass + Nodes[Children + 3]->Mass;

        Nodes[Parents[i]]->CenterOfMass = Vector2Scale(
            Nodes[Parents[i]]->CenterOfMass, 1.f / Nodes[Parents[i]]->Mass );
    }
}

void Quadtree::clear() {
    // Nodes.clear();
    for ( auto& Node : Nodes ) {
        Mb->store( std::move( Node ) );
    }
    Nodes.clear();

    Parents.clear();
}

const std::vector< std::unique_ptr< Quad > >& Quadtree::getNodes() {
    return Nodes;
}
