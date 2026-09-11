#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "algo/basic.h"
#include "core/types.h"
#include "core/utility/flat_hashtable.h"
#include "core/utility/hash.h"

namespace mss {

namespace Structure {

    struct Shape {
        struct Box {
            int size = 0;
        };

        struct ConstraintView {
            int sum = 0;
            std::span<const BoxId> boxIds;
        };

        std::vector<Box> boxes;
        U128 hash = {};

        std::size_t constraintCount() const { return constraints_.size(); }
        ConstraintView constraint(std::size_t i) const {
            const Constraint& c = constraints_[i];
            if (c.count == 0) return {};
            return {c.sum,
                    std::span<const BoxId>(boxIds_.data() + c.offset, c.count)};
        }

        struct Constraint {
            int sum = 0;
            std::uint32_t offset = 0;
            std::uint8_t count = 0;
        };

        std::vector<Constraint> constraints_;
        std::vector<BoxId> boxIds_;
    };

    struct Instance {
        struct Boxes {
            std::vector<CellId> cells;
            std::vector<std::uint16_t> boxOf;

            std::size_t count() const { return boxOf.empty() ? 0 : boxOf.size() - 1; }
            std::size_t cellCount(std::size_t box) const {
                return boxOf[box + 1] - boxOf[box];
            }
        };

        ShapeId shape = -1;
        Boxes boxes;
        std::vector<CellId> constraintCells;
    };

    struct Result {
        std::vector<Instance> components;
        std::vector<CellLocation> cellLoc;
    };

    struct Delta {
        std::vector<ComponentId> removed;
        std::vector<Instance> removedData;
        std::vector<ComponentId> added;
        std::vector<Instance> addedData;
    };

    struct ShapePool {
        ShapeId intern(Shape shape);
        const Shape& get(ShapeId id) const { return shapes_[id]; }
        std::size_t size() const { return shapes_.size(); }

    private:
        std::vector<Shape> shapes_;
        FlatHashTable<U128, ShapeId, U128Hash> index_;
    };

    struct Workspace {
        struct Analyze {
            Grid<char> visited;
            Grid<U128> cellHash;
            std::vector<CellId> cells;
        } analyze;

        struct Update {
            Grid<char> visited;
            Grid<U128> cellHash;
            std::vector<CellId> cells;
            Grid<char> dirty;
            std::vector<CellId> dirtyCells;
            std::vector<char> removed;
            std::vector<Instance> staged;
        } update;

        FlatHashTable<U128, BoxId, U128Hash> hashBox;
        std::vector<BoxId> boxOfCells;
        std::vector<std::array<CellId, 9>> buckets;
        std::vector<std::uint8_t> bucketSize;
        std::vector<BoxId> boxCursor;
        std::vector<char> boxUsed;
        std::vector<BoxId> allBoxIds;
    };

    Result analyze(const ObservedBoard::Result& board,
                   const Basic::Result& basic, ShapePool& pool);

    Delta update(const ObservedBoard::Result& board,
                 const Basic::Result& basic, Result& result,
                 ShapePool& pool, const ObservedBoard::Delta& updates);

    void applyDelta(Result& result, const Delta& delta, bool reverse = true);

}  // namespace Structure

}  // namespace mss
