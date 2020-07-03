namespace CGAL {

/*!
\relates Polyhedron_3
\ingroup PkgPolyhedronIOFunc

This function reads a polyhedral surface in %Object File Format, OFF,
with file extension <TT>.off</TT>, which is also understood by
Geomview \cgalCite{cgal:p-gmgv16-96}, from the input stream `in` and
appends it to the polyhedral surface \f$ P\f$. Only the point coordinates
and facets from the input stream are used to build the polyhedral
surface. Neither normal vectors nor color attributes are evaluated.

For OFF an ASCII and a binary format exist. The stream detects the
format automatically and can read both.

\sa `CGAL::Polyhedron_3<Traits>`
\sa `CGAL::Polyhedron_incremental_builder_3<HDS>`
\sa \link PkgPolyhedronIOFunc `operator<<(std::ostream&, Polyhedron_3<PolyhedronTraits_3>&)`\endlink

This function overloads the generic function \link PkgBGLIoFuncsOFF `read_OFF(std::istream&, FaceGraph)`\endlink.

\cgalHeading{Implementation}

This operator is implemented using the modifier mechanism for
polyhedral surfaces and the `Polyhedron_incremental_builder_3`
class, which allows the construction in a single, efficient scan pass
of the input and handles also all the possible flexibility of the
polyhedral surface.

\note Before \cgal 5.0 this function has set the `ios::badbit` of the input stream `in`
when the file contained 0 vertices.

*/
template <class PolyhedronTraits_3>
bool read_OFF( std::istream& in, Polyhedron_3<PolyhedronTraits_3>& P);

/*!
\relates Polyhedron_3
\deprecated This function is deprecated since \cgal 5.2,
            \link PkgPolyhedronIOFunc `CGAL::read_OFF(std::ostream&, Polyhedron_3<PolyhedronTraits_3>&)` \endlink should be used instead.
*/
template <class PolyhedronTraits_3>
bool read_off( std::ostream& out, Polyhedron_3<PolyhedronTraits_3>& P);

/*!
\relates Polyhedron_3
\ingroup PkgPolyhedronIOFunc
calls \link read_OFF() `read_OFF(in, P)` \endlink.
*/
template <class PolyhedronTraits_3>
std::istream& operator>>( std::istream& in, Polyhedron_3<PolyhedronTraits_3>& P);

/*!
\relates Polyhedron_3
\ingroup PkgPolyhedronIOFunc

This function writes the polyhedral surface \f$P\f$ to the output
stream `out` using the %Object File Format, OFF, with file extension
<TT>.off</TT>, which is also understood by GeomView \cgalCite{cgal:p-gmgv16-96}. The
output is in ASCII format. From the polyhedral surface, only the point
coordinates and facets are written. Neither normal vectors nor color
attributes are used.

For OFF an ASCII and a binary format exist. The format can be selected
with the \cgal modifiers for streams, `set_ascii_mode()` and
`set_binary_mode()` respectively. The modifier `set_pretty_mode()` can be used
to allow for (a few) structuring comments in the output. Otherwise,
the output would be free of comments. The default for writing is ASCII
without comments.

This function overloads the generic function \link PkgBGLIoFuncsOFF `write_OFF(std::istream&,FaceGraph)` \endlink.

\sa `CGAL::Polyhedron_3<Traits>`
\sa `CGAL::Polyhedron_incremental_builder_3<HDS>`
\sa \link PkgPolyhedronIOFunc `operator>>(std::istream& in, Polyhedron_3<PolyhedronTraits_3>& P)` \endlink
*/
template <class PolyhedronTraits_3>
bool write_OFF( std::ostream& out, Polyhedron_3<PolyhedronTraits_3>& P);

/*!
\relates Polyhedron_3
\deprecated This function is deprecated since \cgal 5.2,
            \link PkgPolyhedronIOFunc `CGAL::write_OFF(std::ostream&, Polyhedron_3<PolyhedronTraits_3>&)` \endlink should be used instead.
*/
template <class PolyhedronTraits_3>
bool write_off( std::ostream& out, Polyhedron_3<PolyhedronTraits_3>& P);

/*!
\relates Polyhedron_3
\ingroup PkgPolyhedronIOFunc
calls \link write_OFF() `write_OFF(out, P)` \endlink.
*/
template <class PolyhedronTraits_3>
std::ostream& operator<<( std::ostream& out, Polyhedron_3<PolyhedronTraits_3>& P);

} /* namespace CGAL */

