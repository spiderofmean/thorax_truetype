//===- thorax_truetype.cpp -------------------------------------*- C++ --*-===//
// Copyright 2015  Warren Hunt
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

#include "thorax_truetype.h"

unsigned CharacterMap00::GetGlyphIndexByCodepoint(unsigned codepoint) const {
    return codepoint < 256 ? glyphIndexArray[codepoint] : missingGlyphIndex;
}

unsigned CharacterMap04::GetGlyphIndexByCodepoint(unsigned codepoint) const {
    if (codepoint > 0xffff) return missingGlyphIndex;
    auto startCode = endCode + 1 + segCountX2 / 2;
    auto idDelta = endCode + 1 + segCountX2 / 2 * 2;
    auto idRangeOffset = endCode + 1 + segCountX2 / 2 * 3;

    size_t index = 0;
    // Use the range shift to reduce the search space to a power of 2.
    if (codepoint > endCode[rangeShift / 2]) index += rangeShift / 2;
    // Binary search to find the correct range.
    for (unsigned i = entrySelector, range = searchRange / 4; i; range /= 2, --i)
        if (codepoint > endCode[index + range - 1]) index += range;
    if (codepoint < startCode[index]) return missingGlyphIndex;
    // See documentation + math is 16-bit!
    unsigned offset = idRangeOffset[index];
    if (offset == 0) return (codepoint + idDelta[index]) & 0xffff;
    return idRangeOffset[index + offset / 2 + (codepoint - startCode[index])];
}

unsigned CharacterMap06::GetGlyphIndexByCodepoint(unsigned codepoint) const {
    return codepoint >= firstCode || codepoint - firstCode < entryCount ? glyphIndexArray[codepoint - firstCode]
                                                                        : missingGlyphIndex;
}

unsigned CharacterMap12::GetGlyphIndexByCodepoint(unsigned codepoint) const {
    size_t low = 0, high = nGroups;
    do { // Binary search the right group.
        size_t mid = low + (high - low) / 2;
        if (codepoint > groups[mid].endCharCode)
            low = mid + 1;
        else if (codepoint < groups[mid].startCharCode)
            high = mid;
        else
            return codepoint - groups[mid].startCharCode + groups[mid].startGlyphCode;
    } while (low < high);
    return missingGlyphIndex;
}

const char* CharacterMap::GetUnicodeMapData() const {
    for (size_t i = 0, e = numberSubtables; i != e; ++i)
        if (subtable[i].platformID == PLATFORM_ID_UNICODE || (subtable[i].platformID == PLATFORM_ID_MICROSOFT &&
                                                              (subtable[i].platformSpecificID == MS_EID_UNICODE_BMP ||
                                                               subtable[i].platformSpecificID == MS_EID_UNICODE_FULL)))
            return (const char*)this + subtable[i].offset;
    return nullptr;
}

const Font::TableRecord* Font::GetRecordByTag(const char* tag) const {
    for (size_t i = 0, e = numTables; i != e; ++i)
        if (table[i].tag == tag) return &table[i];
    return nullptr;
}

const Font* FontCollection::GetFontByIndex(size_t index) const {
    return index < numFonts ? (const Font*)((const char*)this + offsetTable[index]) : nullptr;
}

unsigned GlyphData::DecodeGlyphPoints(GlyphPoint* points) const {
    if (numberOfContours < 1) return 0;
    unsigned numPoints = endPtsOfContours[numberOfContours - 1] + 1;
    if (!points) return numPoints;
    auto numInstructions = endPtsOfContours[numberOfContours];
    auto p = (const unsigned char*)(endPtsOfContours + numberOfContours + 1) + numInstructions;

    unsigned char flagCount = 0;
    unsigned char flags = 0;
    for (unsigned i = 0; i < numPoints; i++) {
        if (!flagCount) {
            flags = *p++;
            if (flags & /*REPEAT=*/8) flagCount = *p++;
        } else
            --flagCount;
        points[i].flags = flags;
    }
    int x = 0;
    for (unsigned i = 0; i < numPoints; i++) {
        if (points[i].XIsShortVector()) {
            x += points[i].XIsSameOrPositive() ? *p : -*p;
            p++;
        } else {
            if (!points[i].XIsSameOrPositive()) {
                x += *(const Int16BE*)p;
                p += 2;
            }
        }
        points[i].value_x = (short)x;
    }
    int y = 0;
    for (unsigned i = 0; i < numPoints; i++) {
        if (points[i].YIsShortVector()) {
            y += points[i].YIsSameOrPositive() ? *p : -*p;
            p++;
        } else {
            if (!points[i].YIsSameOrPositive()) {
                y += *(const Int16BE*)p;
                p += 2;
            }
        }
        points[i].value_y = (short)y;
    }
    return numPoints;
}

unsigned GlyphData::DecodeCompoundGlyph(GlyphReference* references) const {
    GlyphReference ref;
    auto p = (char*)endPtsOfContours;
    unsigned count = 0;
    do {
        ref.flags = *(UInt16BE*)p;
        ref.glyphIndex = *(UInt16BE*)(p + 2);
        p += 4;
        if (ref.flags & GlyphReference::ARG_1_AND_2_ARE_WORDS) {
            if (ref.flags & GlyphReference::ARGS_ARE_XY_VALUES) {
                ref.offset_x = *(Int16BE*)p;
                ref.offset_y = *(Int16BE*)(p + 2);
            } else {
                ref.argument1 = *(UInt16BE*)p;
                ref.argument2 = *(UInt16BE*)(p + 2);
            }
            p += 4;
        } else {
            if (ref.flags & GlyphReference::ARGS_ARE_XY_VALUES) {
                ref.offset_x = *(char*)p;
                ref.offset_y = *(char*)(p + 2);
            } else {
                ref.argument1 = *(unsigned char*)p;
                ref.argument2 = *(unsigned char*)(p + 1);
            }
            p += 2;
        }
        if (ref.flags & GlyphReference::ARGS_ARE_XY_VALUES)
            ref.argument1 = ref.argument2 = 0;
        else
            ref.offset_x = ref.offset_y = 0; // TODO: implemente point correspondence
        ref.x_y = ref.y_x = 0;
        if (ref.flags & GlyphReference::WE_HAVE_A_SCALE) {
            ref.x_x = ref.y_y = *(Fixed2_14*)p;
            p += 2;
        } else if (ref.flags & GlyphReference::WE_HAVE_AN_X_AND_Y_SCALE) {
            ref.x_x = *(Fixed2_14*)p;
            ref.y_y = *(Fixed2_14*)(p + 2);
            p += 4;
        } else if (ref.flags & GlyphReference::WE_HAVE_A_TWO_BY_TWO) {
            ref.x_x = *(Fixed2_14*)p;
            ref.y_x = *(Fixed2_14*)(p + 2);
            ref.x_y = *(Fixed2_14*)(p + 4);
            ref.y_y = *(Fixed2_14*)(p + 6);
            p += 8;
        } else
            ref.x_x = ref.y_y = 1;
        if (references) references[count] = ref;
        count++;
    } while (ref.flags & GlyphReference::MORE_COMPONENTS);
    return count;
}

bool FontInfo::Initialize(const Font* font) {
    if (!font) return Clear(), true;
    characterMap = font->GetTableByTag<CharacterMap>("cmap");
    if (!characterMap) return Clear(), false;
    unicodeMapData = characterMap->GetUnicodeMapData();
    if (!unicodeMapData) return Clear(), false;
    switch (*(const UInt16BE*)unicodeMapData) {
        case 0:
            unicodeMapFunction = [](const char* data, unsigned codepoint) {
                return ((const CharacterMap00*)data)->GetGlyphIndexByCodepoint(codepoint);
            };
            break;
        case 4:
            unicodeMapFunction = [](const char* data, unsigned codepoint) {
                return ((const CharacterMap04*)data)->GetGlyphIndexByCodepoint(codepoint);
            };
            break;
        case 6:
            unicodeMapFunction = [](const char* data, unsigned codepoint) {
                return ((const CharacterMap06*)data)->GetGlyphIndexByCodepoint(codepoint);
            };
            break;
        case 12:
            unicodeMapFunction = [](const char* data, unsigned codepoint) {
                return ((const CharacterMap12*)data)->GetGlyphIndexByCodepoint(codepoint);
            };
            break;
        default:
            unicodeMapFunction = [](const char*, unsigned) { return missingGlyphIndex; };
    }
    glpyhData = font->GetTableByTag<char>("glyf");
    if (!glpyhData) return Clear(), false;
    fontHeader = font->GetTableByTag<FontHeader>("head");
    if (!fontHeader) return Clear(), false;
    metricsHeader = font->GetTableByTag<MetricsHeader>("hhea");
    if (!metricsHeader) return Clear(), false;
    horizontalMetrics = (const HorizontalMetric*)font->GetTableByTag<char>("hmtx");
    if (!horizontalMetrics) return Clear(), false;
    leftSideBearings = (const Int16BE*)(horizontalMetrics + metricsHeader->numOfLongHorMetrics);
    if (!leftSideBearings) return Clear(), false;
    // auto kern = font->GetRecordByTag("kern");
    indexToLocation = font->GetTableByTag<char>("loca");
    if (!indexToLocation) return Clear(), false;
    maximumProfile = font->GetTableByTag<MaximumProfile>("maxp");
    if (!maximumProfile) return Clear(), false;
    return true;
}

void FontInfo::Clear() {
    characterMap = nullptr;
    unicodeMapData = nullptr;
    unicodeMapFunction = nullptr;
    fontHeader = nullptr;
    glpyhData = nullptr;
    metricsHeader = nullptr;
    horizontalMetrics = nullptr;
    leftSideBearings = nullptr;
    indexToLocation = nullptr;
    maximumProfile = nullptr;
}

const GlyphData* FontInfo::GetGlyphDataByIndex(unsigned index) const {
    if (fontHeader->indexToLocFormat) {
        auto loca = (const UInt32BE*)indexToLocation;
        return loca[index] != loca[index + 1] ? (const GlyphData*)(glpyhData + loca[index]) : nullptr;
    }
    auto loca = (const UInt16BE*)indexToLocation;
    return loca[index] != loca[index + 1] ? (const GlyphData*)(glpyhData + loca[index] * 2) : nullptr;
}

HorizontalMetric FontInfo::GetHorizontalMetricByIndex(unsigned index) const {
    // https://developer.apple.com/fonts/TrueType-Reference-Manual/RM06/Chap6hmtx.html
    auto numLongMetrics = (unsigned)metricsHeader->numOfLongHorMetrics;
    if (index < numLongMetrics) return horizontalMetrics[index];
    HorizontalMetric metric;
    metric.advanceWidth = horizontalMetrics[numLongMetrics - 1].advanceWidth;
    metric.leftSideBearing = leftSideBearings[index - numLongMetrics];
    return metric;
}

//==============================================================================
// Creation routines.
//==============================================================================

const Font* Font::AsFont(const char* fileData) {
    auto font = (const Font*)fileData;
    // Check for : TrueType 1 || OpenType with CFF || OpenType 1.0
    return font->versionTag == "1\0\0\0" || font->versionTag == "OTTO" || font->versionTag == "\0\1\0\0" ? font
                                                                                                         : nullptr;
}

const FontCollection* FontCollection::AsFontCollection(const char* fileData) {
    auto collection = (const FontCollection*)fileData;
    return collection->versionTag == "ttcf" && (collection->version == 0x00010000 || collection->version == 0x00020000)
               ? collection
               : nullptr;
}
