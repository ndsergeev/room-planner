//
//  GFPConverter.swift
//  room-planner
//
//  Created by Nikita Sergeev on 28/5/2025.
//

import RoomPlan
import simd

typealias Vector2 = SIMD2<Float>
typealias Vector3 = SIMD3<Float>

extension FloatingPoint {
    func isAlmostEqual(to other: Self, tolerance: Self) -> Bool {
        return abs(self - other) <= tolerance
    }
}

extension GFP.FVector3 {
    func isAlmostEqual(to other: GFP.FVector3, tolerance: Float) -> Bool {
        return
            self.X.isAlmostEqual(to: other.X, tolerance: tolerance)
            && self.Y.isAlmostEqual(to: other.Y, tolerance: tolerance)
            && self.Z.isAlmostEqual(to: other.Z, tolerance: tolerance)
    }
}

extension SIMD where Scalar: FloatingPoint {
    func isAlmostEqual(to other: Self, tolerance: Scalar) -> Bool {
        for i in 0..<Self.scalarCount {
            if abs(self[i] - other[i]) > tolerance {
                return false
            }
        }
        return true
    }
}

struct Vertex {
    var ID: Int
    var Location: SIMD3<Float>
}

struct Wall {
    var ID: Int
    var Vertices: [Int]
}

class Building {
    //Consts:
    public let wallThicknessMeters: Float = 0.5
    public let tolerance: Float = 1e-2  // 1 is 1m
    public let unitScale: Float = 100.0  // meters to cm

    //Fields:
    public var vertices: [Vertex] = []
    public var walls: [Wall] = []
    public var internalWalls: [[Int]] = []
    public var externalWalls: [Int] = []

    //GFP
    public var building: GFP.Building?
    public var verticesGFP: [GFP.Vertex] = []
    public var wallsGFP: [GFP.Wall] = []
    public var internalWallsGFP: [[String]] = []  //TODO: implement rooms
    public var externalWallsGFP: [String] = []
    public var height: Float = -Float.greatestFiniteMagnitude

    //TODO: $ndsergeev$ define fields that use raw data types instead of GFP as GFP should be export only

    public var minExtent: SIMD3<Float> = SIMD3(
        x: Float.greatestFiniteMagnitude,
        y: Float.greatestFiniteMagnitude,
        z: Float.greatestFiniteMagnitude,
    )

    private var vertexCounter: Int = 1
    private var wallCounter: Int = 1

    public func nextVertexID() -> Int {
        defer { vertexCounter += 1 }
        return vertexCounter
    }

    public func nextWallID() -> Int {
        defer { wallCounter += 1 }
        return wallCounter
    }

    public func printBuildingAsJSON(from building: GFP.Building) {
        let encoder = JSONEncoder()
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        if let data = try? encoder.encode(building),
            let json = String(data: data, encoding: .utf8)
        {
            print(json)
        } else {
            print("Could not serialize into JSON.")
        }
    }

    private func reset() {
        minExtent = SIMD3(
            x: Float.greatestFiniteMagnitude,
            y: Float.greatestFiniteMagnitude,
            z: Float.greatestFiniteMagnitude
        )

        vertexCounter = 1
        wallCounter = 1
        vertices = []
        walls = []
        internalWalls = [[]]
        externalWalls = []
    }

    public func makeWalls(from capturedRoom: CapturedRoom) {
        func subMakeWalls(adjustLocation: Bool) {
            reset()

            makeInternalWalls(from: capturedRoom, rotate: adjustLocation)

            makeExternalWalls(
                toVert: &vertices,
                toWalls: &walls,
                toEWalls: &externalWalls
            )
            swapAxesAndRescale(vertices: &vertices)
        }

        //Original
        subMakeWalls(adjustLocation: false)
        export()
        validate(vertices: &verticesGFP)

        print("\nRotated:\n")

        //Rotated
        subMakeWalls(adjustLocation: true)
        export()
        validate(vertices: &verticesGFP)
    }

    private func validate(vertices: inout [GFP.Vertex]) {
        var vDict: [String: GFP.FVector3] = [:]
        for v in vertices {
            let loc = v.Location
            let key = "\(loc.X)/\(loc.Y)/\(loc.Z)"
            if vDict[key] != nil {
                print("Duplicate for: \(v.ID)")
            } else {
                vDict[key] = loc
            }
        }
    }

    //TODO: double check the option to separate scanned data to rooms
    // let data = StructureBuilder(options: StructureBuilder.ConfigurationOptions.)
    // data.capturedStructure(from: [capturedRoom])
    private func makeInternalWalls(
        from capturedRoom: CapturedRoom,
        rotate: Bool
    ) {
        var worldFloorVerts: [SIMD3<Float>] = []
        if capturedRoom.floors.count > 1 {
            print("Multiple floors scanned: \(capturedRoom.floors.count)")
        }

        for floor in capturedRoom.floors {
            let ft = floor.transform
            for fv in floor.polygonCorners {
                let f4 = ft * SIMD4(fv.x, fv.y, fv.z, 1)
                worldFloorVerts.append(SIMD3(f4.x, f4.y, f4.z))
            }
        }

        var tempWalls: [[SIMD3<Float>]] = []
        for wall in capturedRoom.walls {
            let dims = wall.dimensions
            let hw = dims.x / 2
            height = max(height, dims.y)
            let t = wall.transform
            let center = SIMD3(t.columns.3.x, t.columns.3.y, t.columns.3.z)
            let rot2 = simd_float2x2([
                SIMD2(t.columns.0.x, t.columns.0.z),
                SIMD2(t.columns.2.x, t.columns.2.z),
            ])
            let halfW = SIMD2(hw, 0)
            let c2 = SIMD2(center.x, center.z)
            let p1 = SIMD3<Float>(
                (c2 + rot2 * halfW).x,
                center.y - dims.y / 2,
                (c2 + rot2 * halfW).y
            )
            let p2 = SIMD3<Float>(
                (c2 - rot2 * halfW).x,
                center.y - dims.y / 2,
                (c2 - rot2 * halfW).y
            )
            tempWalls.append([p1, p2])
            minExtent = simd.min(minExtent, p1)
            minExtent = simd.min(minExtent, p2)
        }

        guard tempWalls.count >= 3 || !BuildFlags.includeWalls else {
            print("Not enough walls: \(tempWalls.count)")
            return
        }

        var remaining = Array(zip(tempWalls, 0..<tempWalls.count))
        var vertexLookup: [(point: SIMD3<Float>, id: Int)] = []

        func getOrCreateVertex(at p: SIMD3<Float>) -> Int {
            if let found = vertexLookup.first(
                where: { $0.point.isAlmostEqual(to: p, tolerance: tolerance) }
            ) {
                return found.id
            }
            let newID = nextVertexID()
            vertices.append(Vertex(ID: newID, Location: p))
            vertexLookup.append((point: p, id: newID))
            return newID
        }

        let (firstSeg, _) = remaining.removeFirst()
        let leftPt = firstSeg[0]
        let rightPt = firstSeg[1]
        let leftID = getOrCreateVertex(at: leftPt)
        let rightID = getOrCreateVertex(at: rightPt)

        var ws: [Wall] = []
        ws.append(Wall(ID: nextWallID(), Vertices: [leftID, rightID]))
        var lastVertexID = rightID
        var curRight = rightPt
        let firstVertexID = leftID

        var didCloseLoop = false
        while !didCloseLoop && !remaining.isEmpty {
            if let matchIdx = remaining.firstIndex(
                where: { (seg, _) in
                    seg.contains(where: {
                        $0.isAlmostEqual(to: curRight, tolerance: tolerance)
                    })
                }
            ) {
                let (seg, _) = remaining.remove(at: matchIdx)
                let nextPt =
                    seg[0].isAlmostEqual(to: curRight, tolerance: tolerance)
                    ? seg[1]
                    : seg[0]

                if nextPt.isAlmostEqual(to: leftPt, tolerance: tolerance) {
                    ws.append(
                        Wall(
                            ID: nextWallID(),
                            Vertices: [lastVertexID, firstVertexID]
                        )
                    )
                    didCloseLoop = true
                    break
                }

                let newID = getOrCreateVertex(at: nextPt)
                ws.append(
                    Wall(ID: nextWallID(), Vertices: [lastVertexID, newID])
                )
                lastVertexID = newID
                curRight = nextPt
            } else {
                // TODO: implement case for incomplete scan
                print(
                    "Incomplete scan – couldn’t find next wall for point \(curRight)"
                )
                break
            }
        }

        offsetVertices(vertices: &vertices, offset: minExtent)
        if rotate {
            rotateVertices(vertices: &vertices, tolerance: tolerance)
        }
        if BuildFlags.includeWalls {
            walls = ws
            internalWalls[0] = ws.map { $0.ID }
        } else {
            walls = []
        }
    }

    /// Offsets all vertices to offset, it is needed to, for instance, make all vertices lie in the positive space.
    /// - simple example is to make sure there is 0 used on Z or a value that's more than 0;
    public func offsetVertices(
        vertices: inout [Vertex],
        offset: SIMD3<Float>
    ) {
        for index in vertices.indices {
            let position = vertices[index].Location
            vertices[index].Location = SIMD3<Float>(
                x: position.x - offset.x,
                y: position.y - offset.y,
                z: position.z - offset.z
            )
        }
    }

    private func rotateVertices(vertices: inout [Vertex], tolerance: Float) {
        func checkAndRotateIfNeeded(
            verts: inout [SIMD3<Float>],
            original: inout [Vertex],
            leftIndex: Int,
            centerIndex: Int,
            rightIndex: Int,
            tolerance: Float
        ) -> Bool {
            let L = verts[leftIndex]
            let C = verts[centerIndex]
            let R = verts[rightIndex]

            let v1 = simd_normalize(C - L)
            let v2 = simd_normalize(R - C)
            let d = simd_dot(v1, v2)

            // check if it is 90°
            if abs(d).isAlmostEqual(to: 0, tolerance: tolerance) {
                // turn direction in XZ plane
                let v1_2D = SIMD2<Float>(v1.x, v1.z)
                let v2_2D = SIMD2<Float>(v2.x, v2.z)
                let cross = v1_2D.x * v2_2D.y - v1_2D.y * v2_2D.x

                // base vector to align with
                let reference = SIMD2<Float>(1, 0)
                let dotRef = simd_dot(reference, v1_2D)
                let clamped = max(-1, min(1, dotRef))
                let angle = acos(clamped)

                let signedAngle = cross > 0 ? angle : -angle

                let anchor = C
                let quat = simd_quaternion(-signedAngle, SIMD3<Float>(0, 1, 0))

                // rotate verts about C(anchor):
                for i in verts.indices {
                    let delta = verts[i] - anchor
                    verts[i] = quat.act(delta) + anchor
                }

                for i in 0..<original.count {
                    original[i].Location = verts[i]
                }

                return true
            }

            return false
        }

        var verts: [SIMD3<Float>] = vertices.map {
            SIMD3<Float>($0.Location.x, $0.Location.y, $0.Location.z)
        }

        let count = verts.count
        if count < 4 {
            print("Not enough vertices: \(count)")
            return
        }

        if checkAndRotateIfNeeded(
            verts: &verts,
            original: &vertices,
            leftIndex: count - 1,
            centerIndex: 0,
            rightIndex: 1,
            tolerance: tolerance
        ) {
            return
        }

        for vi in 0..<(count - 2) {
            if checkAndRotateIfNeeded(
                verts: &verts,
                original: &vertices,
                leftIndex: vi,
                centerIndex: vi + 1,
                rightIndex: vi + 2,
                tolerance: tolerance
            ) {
                return
            }
        }

        if checkAndRotateIfNeeded(
            verts: &verts,
            original: &vertices,
            leftIndex: count - 2,
            centerIndex: count - 1,
            rightIndex: 0,
            tolerance: tolerance
        ) {
            return
        }
    }

    private func makeExternalWalls(
        toVert vertices: inout [Vertex],
        toWalls walls: inout [Wall],
        toEWalls externalWalls: inout [Int]
    ) {
        var minX = Float.greatestFiniteMagnitude
        var maxX = -Float.greatestFiniteMagnitude
        var minZ = Float.greatestFiniteMagnitude
        var maxZ = -Float.greatestFiniteMagnitude

        for v in vertices {
            let p = v.Location
            minX = min(minX, p.x)
            maxX = max(maxX, p.x)
            minZ = min(minZ, p.z)
            maxZ = max(maxZ, p.z)
        }

        let offset: Float = wallThicknessMeters  // meters
        let height: Float = 0.0

        let v1 = Vertex(
            ID: nextVertexID(),
            Location: SIMD3<Float>(
                x: minX - offset,
                y: height,
                z: minZ - offset
            )
        )
        let v2 = Vertex(
            ID: nextVertexID(),
            Location: SIMD3<Float>(
                x: maxX + offset,
                y: height,
                z: minZ - offset
            )
        )
        let v3 = Vertex(
            ID: nextVertexID(),
            Location: SIMD3<Float>(
                x: maxX + offset,
                y: height,
                z: maxZ + offset
            )
        )
        let v4 = Vertex(
            ID: nextVertexID(),
            Location: SIMD3<Float>(
                x: minX - offset,
                y: height,
                z: maxZ + offset
            )
        )
        vertices.append(contentsOf: [v1, v2, v3, v4])

        let e1 = Wall(ID: nextWallID(), Vertices: [v1.ID, v2.ID])
        let e2 = Wall(ID: nextWallID(), Vertices: [v2.ID, v3.ID])
        let e3 = Wall(ID: nextWallID(), Vertices: [v3.ID, v4.ID])
        let e4 = Wall(ID: nextWallID(), Vertices: [v4.ID, v1.ID])
        walls.append(contentsOf: [e1, e2, e3, e4])

        externalWalls = [e1.ID, e2.ID, e3.ID, e4.ID]
    }

    ///Applies scaling that is used in the exported format and swaps Z and Y coordinates
    private func swapAxesAndRescale(vertices: inout [Vertex]) {
        minExtent *= unitScale

        (minExtent.y, minExtent.z) = (minExtent.z, minExtent.y)  // SWAP

        for i in vertices.indices {
            let v = vertices[i].Location
            vertices[i].Location = SIMD3<Float>(
                x: v.x * unitScale,
                y: v.z * unitScale,  // SWAP
                z: v.y * unitScale,  // SWAP
            )
        }
    }

    // TODO: separate exporting part in extension of the class
    private func convert2GFP() {
        verticesGFP = vertices.map { vertex in
            let loc = vertex.Location
            return GFP.Vertex(
                ID: "VertexId\(vertex.ID)",
                Location: GFP.FVector3(
                    X: loc.x,
                    Y: loc.y,
                    Z: loc.z
                )
            )
        }

        wallsGFP = walls.map { wall in
            let verts = wall.Vertices.map { vID in
                "VertexId\(vID)"
            }
            return GFP.Wall(
                ID: "WallId\(wall.ID)",
                Vertices: verts
            )
        }

        externalWallsGFP = externalWalls.map { wID in
            "WallId\(wID)"
        }

        internalWallsGFP = internalWalls.map { wallIDs in
            wallIDs.map { wID in
                "WallId\(wID)"
            }
        }
    }

    // TODO: separate exporting part in extension of the class
    public func export() {
        convert2GFP()

        let room: GFP.Room = GFP.Room(
            ID: "RoomId\(1)",
            Type: "Room",
            Walls: internalWallsGFP[0]
        )

        let unit: GFP.Unit = GFP.Unit(
            Location: GFP.FVector3(X: 0, Y: 0, Z: 0),
            Rotation: GFP.FRotation(Pitch: 0, Roll: 0, Yaw: 0),
            Scale: GFP.FVector3(X: 1, Y: 1, Z: 1),
            Rooms: [room]
        )

        let floor: GFP.Floor = GFP.Floor(
            Number: 1,
            Height: height,
            Location: GFP.FVector3(X: 0, Y: 0, Z: 0),
            Rotation: GFP.FRotation(Pitch: 0, Roll: 0, Yaw: 0),
            Scale: GFP.FVector3(X: 1, Y: 1, Z: 1),
            Units: [unit],
            Walls: wallsGFP,
            ExteriorWalls: externalWallsGFP
        )

        building = GFP.Building(
            Name: "BuildingTest",
            Floors: [floor],
            Vertices: verticesGFP,
            Scale: GFP.FVector3(X: 1, Y: 1, Z: 1)
        )

        //TODO: implement it as a proper export of json so it can be sent over
        printBuildingAsJSON(from: building!)
    }
}
