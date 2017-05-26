//===- thorax_truetype.h ---------------------------------------*- C++ --*-===//
// Copyright 2017  Warren Hunt
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//===----------------------------------------------------------------------===//

#pragma once

// https://www.microsoft.com/typography/otspec/otff.htm

// TODO: File bug on stbtt_GetFontOffsetForIndex (+14)
// TODO: separate out a meaningful API

//==============================================================================
// Dealing with big-endien.
//==============================================================================

extern "C" unsigned short __cdecl _byteswap_ushort(unsigned short);
extern "C" unsigned long __cdecl _byteswap_ulong(unsigned long);

struct UInt16BE {
    UInt16BE() {}
    explicit UInt16BE(unsigned short a) : bigEndian(_byteswap_ushort(a)) {}
    operator unsigned short() const { return _byteswap_ushort(bigEndian); }
    bool operator==(UInt16BE a) const { return bigEndian == a.bigEndian; }
    unsigned short bigEndian;
};

struct Int16BE {
    static inline short byteswap(short x) { return (short)_byteswap_ushort((unsigned short)x); }
    Int16BE() {}
    explicit Int16BE(short a) : bigEndian(byteswap(a)) {}
    operator short() const { return byteswap(bigEndian); }
    bool operator==(UInt16BE a) const { return bigEndian == a.bigEndian; }
    short bigEndian;
};

struct UInt32BE {
    static inline unsigned byteswap(unsigned x) { return (unsigned)_byteswap_ulong((unsigned long)x); }
    UInt32BE() {}
    explicit UInt32BE(unsigned a) : bigEndian(byteswap(a)) {}
    operator unsigned() const { return byteswap(bigEndian); }
    bool operator==(UInt32BE a) const { return bigEndian == a.bigEndian; }
    unsigned bigEndian;
};

struct Fixed16_16 {
    operator double() const { return _byteswap_ulong(bigEndian) * (1.0 / 65536); }
    bool operator==(Fixed16_16 a) const { return bigEndian == a.bigEndian; }
    unsigned long bigEndian;
};

struct Fixed2_14 {
    operator float() const { return _byteswap_ushort(bigEndian) * (1.0f / 16384); }
    bool operator==(Fixed2_14 a) const { return bigEndian == a.bigEndian; }
    unsigned short bigEndian;
};

struct Tag {
    bool operator==(const char* tag) const { return value == *(const unsigned*)tag; }
    unsigned value;
};

//==============================================================================
// Glyph points and decoding.
//==============================================================================

struct GlyphPoint {
    // https://developer.apple.com/fonts/TrueType-Reference-Manual/RM06/Chap6glyf.html
    unsigned char IsOnCurve() const { return flags & 1; };
    unsigned char XIsShortVector() const { return flags & 2; };
    unsigned char YIsShortVector() const { return flags & 4; };
    unsigned char XIsSameOrPositive() const { return flags & 16; };
    unsigned char YIsSameOrPositive() const { return flags & 32; };
    short value_x;
    short value_y;
    unsigned char flags;
};

struct GlyphReference {
    enum {
        ARG_1_AND_2_ARE_WORDS = 1 << 0,    // If set, the arguments are words; If not set, they are bytes.
        ARGS_ARE_XY_VALUES = 1 << 1,       // If set, the arguments are xy values; If not set, they are points.
        ROUND_XY_TO_GRID = 1 << 2,         // If set, round the xy values to grid.
        WE_HAVE_A_SCALE = 1 << 3,          // If set, there is a simple scale for the component.
        MORE_COMPONENTS = 1 << 5,          // If set, at least one additional glyph follows this one.
        WE_HAVE_AN_X_AND_Y_SCALE = 1 << 6, // If set, the x direction will use a different scale than the y direction.
        WE_HAVE_A_TWO_BY_TWO = 1 << 7,     // If set, there is a 2-by-2 transformation used to scale the component.
        WE_HAVE_INSTRUCTIONS = 1 << 8, // If set, instructions for the component character follow the last component.
        USE_MY_METRICS = 1 << 9,       // Use metrics from this component for the compound glyph.
        OVERLAP_COMPOUND = 1 << 10,    // If set, the components of this compound glyph overlap.
    };

    unsigned short flags;
    unsigned short glyphIndex;
    unsigned short argument1;
    unsigned short argument2;
    short offset_x;
    short offset_y;
    float x_x, x_y;
    float y_x, y_y;
};

//==============================================================================
// Character Map Data Structures
//==============================================================================

constexpr unsigned missingGlyphIndex = 0;

struct CharacterMap00 {
    unsigned GetGlyphIndexByCodepoint(unsigned codepoint) const;

    UInt16BE format;                    // Set to 0
    UInt16BE length;                    // Length in bytes of the subtable (set to 262 for format 0)
    UInt16BE language;                  // Language code (see above)
    unsigned char glyphIndexArray[256]; // An array that maps character codes to glyph index values
};

struct CharacterMap04 {
    unsigned GetGlyphIndexByCodepoint(unsigned codepoint) const;

    UInt16BE format;                // Format number is set to 4
    UInt16BE length;                // Length of subtable in bytes
    UInt16BE language;              // Language code (see above)
    UInt16BE segCountX2;            // 2 * segCount
    UInt16BE searchRange;           // 2 * (2**FLOOR(log2(segCount)))
    UInt16BE entrySelector;         // log2(searchRange/2)
    UInt16BE rangeShift;            // (2 * segCount) - searchRange
    UInt16BE endCode[/*segCount*/]; // Ending character code for each segment, last = 0xFFFF.

    // The following fields exist but can't be represented in C/C++ because endCode is variable width.
    // UInt16BE reservedPad;               // This value should be zero
    // UInt16BE startCode[segCount];       // Starting character code for each segment
    // UInt16BE idDelta[segCount];         // Delta for all character codes in segment
    // UInt16BE idRangeOffset[segCount];   // Offset in bytes to glyph indexArray, or 0
    // UInt16BE glyphIndexArray[variable]; // Glyph index array
};

struct CharacterMap06 {
    unsigned GetGlyphIndexByCodepoint(unsigned codepoint) const;

    UInt16BE format;            // Format number is set to 6
    UInt16BE length;            // Length in bytes
    UInt16BE language;          // Language code (see above)
    UInt16BE firstCode;         // First character code of subrange
    UInt16BE entryCount;        // Number of character codes in subrange
    UInt16BE glyphIndexArray[]; // Array of glyph index values for character codes in the range
};

struct CharacterMap12 {
    struct Group {
        UInt32BE startCharCode;  // First character code in this group
        UInt32BE endCharCode;    // Last character code in this group
        UInt32BE startGlyphCode; // Glyph index corresponding to the starting character code; subsequent charcters are
                                 // mapped to sequential glyphs
    };

    unsigned GetGlyphIndexByCodepoint(unsigned codepoint) const;

    Fixed16_16 format; // Subtable format; set to 12.0
    UInt32BE length;   // Byte length of this subtable (including the header)
    UInt32BE language; // Language code (see above)
    UInt32BE nGroups;  // Number of groupings which follow
    Group groups[];
};

struct CharacterMap {
    enum { PLATFORM_ID_UNICODE = 0, PLATFORM_ID_MAC = 1, PLATFORM_ID_MICROSOFT = 3 };

    enum {
        MS_EID_SYMBOL = 0,
        MS_EID_UNICODE_BMP = 1,
        MS_EID_SHIFTJIS = 2,
        MS_EID_PRC = 3,
        MS_EID_BIGFIVE = 4,
        MS_EID_JOHAB = 5,
        MS_EID_UNICODE_FULL = 10
    };

    struct CharacterMapSubtable {
        UInt16BE platformID;         // Platform identifier.
        UInt16BE platformSpecificID; // Platform-specific encoding identifier.
        UInt32BE offset;             // Offset of the mapping table.
    };

    const char* GetUnicodeMapData() const;

    UInt16BE version; // 0
    UInt16BE numberSubtables;
    CharacterMapSubtable subtable[];
};

//==============================================================================
//
//==============================================================================

#pragma pack(push, 1)
struct FontHeader {
    Fixed16_16 version;          // 0x00010000 if (version 1.0)
    Fixed16_16 fontRevision;     // set by font manufacturer
    UInt32BE checkSumAdjustment; //
    UInt32BE magicNumber;        // set to 0x5F0F3CF5
    UInt16BE flags;              //
    UInt16BE unitsPerEm;         // range from 64 to 16384
    long long created;           // international date
    long long modified;          // international date
    Int16BE xMin;                // for all glyph bounding boxes
    Int16BE yMin;                // for all glyph bounding boxes
    Int16BE xMax;                // for all glyph bounding boxes
    Int16BE yMax;                // for all glyph bounding boxes
    UInt16BE macStyle;           //
    UInt16BE lowestRecPPEM;      // smallest readable size in pixels
    Int16BE fontDirectionHint;   //
    Int16BE indexToLocFormat;    // 0 for short offsets, 1 for long
    Int16BE glyphDataFormat;     // 0 for current format
};
#pragma pack(pop)

struct MetricsHeader {
    Fixed16_16 version;           // 0x00010000 (1.0)
    Int16BE ascent;               // Distance from baseline of highest ascender
    Int16BE descent;              // Distance from baseline of lowest descender
    Int16BE lineGap;              // typographic line gap
    UInt16BE advanceWidthMax;     // must be consistent with horizontal metrics
    Int16BE minLeftSideBearing;   // must be consistent with horizontal metrics
    Int16BE minRightSideBearing;  // must be consistent with horizontal metrics
    Int16BE xMaxExtent;           // max(lsb + (xMax-xMin))
    Int16BE caretSlopeRise;       // used to calculate the slope of the caret (rise/run) set to 1 for vertical caret
    Int16BE caretSlopeRun;        // 0 for vertical
    Int16BE caretOffset;          // set value to 0 for non-slanted fonts
    Int16BE reserved[4];          // set value to 0, 0, 0, 0
    Int16BE metricDataFormat;     // 0 for current format
    UInt16BE numOfLongHorMetrics; // number of advance widths in metrics table
};

struct HorizontalMetric {
    UInt16BE advanceWidth;
    Int16BE leftSideBearing;
};

struct MaximumProfile {
    Fixed16_16 version;             // 0x00010000 (1.0)
    UInt16BE numGlyphs;             // the number of glyphs in the font
    UInt16BE maxPoints;             // points in non-compound glyph
    UInt16BE maxContours;           // contours in non-compound glyph
    UInt16BE maxCompositePoints;    // points in compound glyph
    UInt16BE maxCompositeContours;  // contours in compound glyph
    UInt16BE maxZones;              // set to 2
    UInt16BE maxTwilightPoints;     // points used in Twilight Zone (Z0)
    UInt16BE maxStorage;            // number of Storage Area locations
    UInt16BE maxFunctionDefs;       // number of FDEFs
    UInt16BE maxInstructionDefs;    // number of IDEFs
    UInt16BE maxStackElements;      // maximum stack depth
    UInt16BE maxSizeOfInstructions; // byte count for glyph instructions
    UInt16BE maxComponentElements;  // number of glyphs referenced at top level
    UInt16BE maxComponentDepth;     // levels of recursion, set to 0 if font has only simple glyphs
};

struct GlyphData {
    unsigned DecodeGlyphPoints(GlyphPoint* points) const;
    unsigned DecodeCompoundGlyph(GlyphReference* references) const;

    Int16BE numberOfContours; // negative implies compound glyph
    Int16BE xMin;             // Minimum x for coordinate data
    Int16BE yMin;             // Minimum y for coordinate data
    Int16BE xMax;             // Maximum x for coordinate data
    Int16BE yMax;             // Maximum y for coordinate data
    UInt16BE endPtsOfContours[];
};

struct CompoundGlyphData {
    // https://developer.apple.com/fonts/TrueType-Reference-Manual/RM06/Chap6glyf.html
    UInt16BE flags;      // Component flag
    UInt16BE glyphIndex; // Glyph index of component

    // int16, uint16, int8 or uint8 argument1 X-offset for component or point number.
    // int16, uint16, int8 or uint8 argument2 Y-offset for component or point number.
    // Types depends on bits 0 and 1 in component flags.
};

//==============================================================================
// Font and Font Collection Structures.
//==============================================================================

struct Font {
    struct TableRecord {
        Tag tag;
        UInt32BE checkSum; // We ignore the checksum.
        UInt32BE offset;
        UInt32BE length;
    };

    const TableRecord* GetRecordByTag(const char* tag) const;
    template <typename T> const T* GetTableByTag(const char* tag) const {
        auto recordTable = GetRecordByTag(tag);
        if (!recordTable) return nullptr;
        return (const T*)((const char*)this + recordTable->offset);
    }

    static const Font* AsFont(const char* fileData);

    Tag versionTag;
    UInt16BE numTables;     // Number of tables.
    UInt16BE searchRange;   // (Maximum power of 2 <= numTables) x 16.
    UInt16BE entrySelector; // Log2(maximum power of 2 <= numTables).
    UInt16BE rangeShift;    // NumTables x 16-searchRange.
    TableRecord table[];
};

struct FontCollection {
    const Font* GetFontByIndex(size_t index) const;

    static const FontCollection* AsFontCollection(const char* fileData);

    Tag versionTag;
    UInt32BE version;
    UInt32BE numFonts;
    UInt32BE offsetTable[];
};

//==============================================================================
// FontInfo structure
// Contains all of the tables in the ttf.
//==============================================================================

struct FontInfo {
    FontInfo() { Clear(); }
    bool Initialize(const Font* font);
    void Clear();
    const GlyphData* GetGlyphDataByIndex(unsigned index) const;
    HorizontalMetric GetHorizontalMetricByIndex(unsigned index) const;
    unsigned GetGlyphIndexByCodepoint(unsigned codepoint) const {
        return unicodeMapFunction(unicodeMapData, codepoint);
    }
    const GlyphData* GetGlyphDataByCodepoint(unsigned codepoint) const {
        return GetGlyphDataByIndex(GetGlyphIndexByCodepoint(codepoint));
    }
    HorizontalMetric GetHorizontalMetricByCodepoint(unsigned codepoint) const {
        return GetHorizontalMetricByIndex(GetGlyphIndexByCodepoint(codepoint));
    }

    const CharacterMap* characterMap;
    const char* unicodeMapData;
    unsigned (*unicodeMapFunction)(const char*, unsigned);
    const FontHeader* fontHeader;
    const char* glpyhData;
    const MetricsHeader* metricsHeader;
    const HorizontalMetric* horizontalMetrics;
    const Int16BE* leftSideBearings;
    const char* indexToLocation;
    const MaximumProfile* maximumProfile;
};

//==============================================================================
//
//==============================================================================

#ifdef THORAX_TRUETYPE_IMPL
#include "thorax_truetype.cpp"
#endif
