//
//  GFPConverter.swift
//  room-planner
//
//  Created by Nikita Sergeev on 28/5/2025.
//

import RoomPlan
import simd

class BuildingExporter {
    //Consts:
    private let precision: Float = 1e-2  // 1 is 1m
    private let unitScale: Float = 100.0  // meters to cm

    //Fields:
    private var building: GFP.Building?
    private var vertices: [GFP.Vertex] = []
    private var walls: [GFP.Wall] = []
    private var externalVertices: [GFP.Vertex] = []
    private var internalWalls: [[String]] = []  //TODO: implement rooms
    private var externalWalls: [String] = []

    private var minExtent: SIMD3<Float> = SIMD3(
        x: Float.greatestFiniteMagnitude,
        y: Float.greatestFiniteMagnitude,
        z: Float.greatestFiniteMagnitude,
    )

    private var maxExtent: SIMD3<Float> = SIMD3(
        x: -Float.greatestFiniteMagnitude,
        y: -Float.greatestFiniteMagnitude,
        z: -Float.greatestFiniteMagnitude,
    )

    private var vertexCounter: Int = 1
    private var wallCounter: Int = 1

    private func nextVertexID() -> String {
        defer { vertexCounter += 1 }
        return "VertexId\(vertexCounter)"
    }

    private func nextWallID() -> String {
        defer { wallCounter += 1 }
        return "WallId\(wallCounter)"
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

    func printOBJFloor(from capturedRoom: CapturedRoom) {
        var floorCounter = 0
        for floor in capturedRoom.floors {
            print("o Floor\(floorCounter)")
            let floorTransform = floor.transform
            for floorVert in floor.polygonCorners {
                let floorVert =
                    floorTransform
                    * SIMD4(floorVert.x, floorVert.y, floorVert.z, 1)
                let worldFloorVert = SIMD3(
                    floorVert.x,
                    floorVert.y,
                    floorVert.z
                )
                print(
                    "v \(worldFloorVert.x) \(worldFloorVert.y) \(worldFloorVert.z)"
                )
            }
            floorCounter += 1
            print()
        }
    }

    func printOBJFloorOutline(from capturedRoom: CapturedRoom) {
        var floorCounter = 0
        for floor in capturedRoom.floors {
            print("o Floor\(floorCounter)")
            let floorTransform = floor.transform
            for floorVert in floor.polygonCorners {
                let floorVert =
                    floorTransform
                    * SIMD4(floorVert.x, floorVert.y, floorVert.z, 1)
                let worldFloorVert = SIMD3(
                    floorVert.x,
                    floorVert.y,
                    floorVert.z
                )
                print(
                    "v \(worldFloorVert.x) \(worldFloorVert.y) \(worldFloorVert.z)"
                )
            }
            floorCounter += 1
            print()
        }
    }

    private func reset() {
        vertexCounter = 1
        wallCounter = 1
        vertices = []
        walls = []
    }

    public func makeWalls(from capturedRoom: CapturedRoom) {
        reset()

        //TODO: keep vertices in [SIMD3<Float>] and walls in [Int], use GFP for export only
        makeInternalWalls(from: capturedRoom)
        makeExternalWalls(
            toVert: &vertices,
            toWalls: &walls,
            toEWalls: &externalWalls
        )
        prepareForExport(to: &vertices)

        validateRoom(vertices: &vertices, walls: &walls)
    }

    //TODO: $ndsergeev$, it is not yet completed
    private func validateRoom(vertices: inout [GFP.Vertex], walls: inout [GFP.Wall]) {
        guard !walls.isEmpty else {
            assertionFailure("No walls to validate.")
            return
        }
        
        // Build adjacency graph
        var adjacency: [String: [String]] = [:]
        for wall in walls {
            let v0 = wall.Vertices[0]
            let v1 = wall.Vertices[1]
            adjacency[v0, default: []].append(v1)
            adjacency[v1, default: []].append(v0)
        }
        
        // DFS to find connected component
        var visited: Set<String> = []
        var stack: [String] = [walls[0].Vertices[0]]
        
        while let current = stack.popLast() {
            if visited.contains(current) { continue }
            visited.insert(current)
            stack.append(contentsOf: adjacency[current, default: []])
        }
        
        // Check for unvisited (disconnected) vertices
        let usedVertexIDs = Set(walls.flatMap { $0.Vertices })
        let disconnected = usedVertexIDs.subtracting(visited)
        
        guard !disconnected.isEmpty else {
            print("All walls are connected.")
            return
        }
        
        print("Disconnected vertices: \(Array(disconnected))")
    }

    ///Applies scaling that is used in the exported format and swaps Z and Y coordinates
    private func prepareForExport(to vertices: inout [GFP.Vertex]) {
        minExtent *= unitScale
        maxExtent *= unitScale

        (minExtent.y, minExtent.z) = (minExtent.z, minExtent.y)  // SWAP
        (maxExtent.y, maxExtent.z) = (maxExtent.z, maxExtent.y)  // SWAP

        for i in vertices.indices {
            let v = vertices[i].Location
            vertices[i].Location = GFP.FVector3(
                X: v.X * unitScale,
                Y: v.Z * unitScale,  // SWAP
                Z: v.Y * unitScale,  // SWAP
            )
        }
    }

    private func makeInternalWalls(from capturedRoom: CapturedRoom) {
        let precisionSquared = precision * precision

        var tempWalls: [[GFP.FVector3]] = []
        for wall in capturedRoom.walls {
            let wallTransform = wall.transform
            let size = wall.dimensions

            let hw = size.x / 2
            let hh = size.y / 2
            let hd = size.z / 2

            //TODO; two top vertices should be discarded
            let wallVerts: [SIMD4] = [
                SIMD4(-hw, -hh, -hd, 1),  // 0
                SIMD4(hw, -hh, -hd, 1),  // 1
                SIMD4(hw, hh, -hd, 1),  // 2
                SIMD4(-hw, hh, -hd, 1),  // 3
            ]

            var thisWall: [GFP.FVector3] = []
            for wv in wallVerts {
                let transformed = wallTransform * wv
                let worldWall = SIMD3(
                    transformed.x,
                    transformed.y,
                    transformed.z
                )

                for floor in capturedRoom.floors {
                    let ft = floor.transform
                    for fv in floor.polygonCorners {
                        let f4 = ft * SIMD4(fv.x, fv.y, fv.z, 1)
                        let worldFloor = SIMD3(f4.x, f4.y, f4.z)

                        maxExtent = simd.max(maxExtent, worldWall)
                        minExtent = simd.min(minExtent, worldFloor)

                        // record corner if its within precision limit on the floor
                        if simd_dot(
                            worldWall - worldFloor,
                            worldWall - worldFloor
                        ) < precisionSquared {
                            thisWall.append(
                                GFP.FVector3(
                                    X: worldFloor.x,
                                    Y: worldFloor.y,
                                    Z: worldFloor.z
                                )
                            )
                        }
                    }
                }
            }
            tempWalls.append(thisWall)
        }

        var positionToID = [String: String]()
        for wallPositions in tempWalls {
            for pos in wallPositions {
                let key = "\(pos.X),\(pos.Y),\(pos.Z)"
                if positionToID[key] == nil {
                    positionToID[key] = nextVertexID()
                }
            }
        }

        for (key, vID) in positionToID {
            let comps = key.split(separator: ",").map { Float($0)! }
            let loc = GFP.FVector3(X: comps[0], Y: comps[1], Z: comps[2])
            vertices.append(GFP.Vertex(ID: vID, Location: loc))
        }

        internalWalls.append([])
        for wallLocs in tempWalls {
            let wID = nextWallID()
            let vIDs = wallLocs.map { loc -> String in
                let key = "\(loc.X),\(loc.Y),\(loc.Z)"
                return positionToID[key]!  // must exist
            }
            walls.append(GFP.Wall(ID: wID, Vertices: vIDs))
            internalWalls[0].append(wID)
        }
    }

    //TODO: $ndsergeev$ remove
    private func makeInternalWallsOLD(from capturedRoom: CapturedRoom) {
        let precisionSquared = precision * precision

        var tempWalls: [[GFP.FVector3]] = []
        for wall in capturedRoom.walls {
            let wallTransform = wall.transform
            let size = wall.dimensions

            let hw = size.x / 2
            let hh = size.y / 2
            let hd = size.z / 2

            //TODO; two top vertices should be discarded
            let wallVerts: [SIMD4] = [
                SIMD4(-hw, -hh, -hd, 1),  // 0
                SIMD4(hw, -hh, -hd, 1),  // 1
                SIMD4(hw, hh, -hd, 1),  // 2
                SIMD4(-hw, hh, -hd, 1),  // 3
            ]

            var tempWall: [GFP.FVector3] = []
            for wallVert in wallVerts {
                let wallVert = wallTransform * wallVert
                let worldWallVert = SIMD3(
                    wallVert.x,
                    wallVert.y,
                    wallVert.z
                )

                for floor in capturedRoom.floors {
                    let floorTransform = floor.transform
                    for floorVert in floor.polygonCorners {
                        let floorVert =
                            floorTransform
                            * SIMD4(floorVert.x, floorVert.y, floorVert.z, 1)
                        let worldFloorVert = SIMD3(
                            floorVert.x,
                            floorVert.y,
                            floorVert.z
                        )

                        //Max:, TODO: try simd_max
                        maxExtent.x = max(maxExtent.x, wallVert.x)
                        maxExtent.y = max(maxExtent.y, wallVert.y)
                        maxExtent.z = max(maxExtent.z, wallVert.z)

                        //Min:, TODO: try simd_max
                        minExtent.x = min(minExtent.x, floorVert.x)
                        minExtent.y = min(minExtent.y, floorVert.y)
                        minExtent.z = min(minExtent.z, floorVert.z)

                        let worldDiff = worldWallVert - worldFloorVert
                        if simd_dot(worldDiff, worldDiff) < precisionSquared {
                            let location = GFP.FVector3(
                                X: floorVert.x,
                                Y: floorVert.y,
                                Z: floorVert.z,
                            )
                            tempWall.append(location)
                        }
                    }
                }
            }

            tempWalls.append(tempWall)
        }

        let minOffset = SIMD3<Float>(
            x: minExtent.x,
            y: minExtent.y,
            z: minExtent.z
        )

        for wall in tempWalls {
            let wID = nextWallID()

            var wallStrings: [String] = []
            for vertex in wall {
                let vID = nextVertexID()
                wallStrings.append(vID)
                let location = GFP.Vertex(
                    ID: vID,
                    Location: GFP.FVector3(
                        X: (vertex.X - minOffset.x),
                        Y: (vertex.Y - minOffset.y),
                        Z: (vertex.Z - minOffset.z)
                    )
                )

                vertices.append(location)
            }

            walls.append(GFP.Wall(ID: wID, Vertices: wallStrings))
        }
    }

    private func makeExternalWalls(
        toVert vertices: inout [GFP.Vertex],
        toWalls walls: inout [GFP.Wall],
        toEWalls externalWalls: inout [String]
    ) {
        var minX = Float.greatestFiniteMagnitude
        var maxX = -Float.greatestFiniteMagnitude
        var minZ = Float.greatestFiniteMagnitude
        var maxZ = -Float.greatestFiniteMagnitude

        for v in vertices {
            let p = v.Location
            minX = min(minX, p.X)
            maxX = max(maxX, p.X)
            minZ = min(minZ, p.Z)
            maxZ = max(maxZ, p.Z)
        }

        let offset: Float = 0.2  // meters
        let height: Float = 0.0

        let v1 = GFP.Vertex(
            ID: nextVertexID(),
            Location: GFP.FVector3(
                X: minX - offset,
                Y: height,
                Z: minZ - offset
            )
        )
        let v2 = GFP.Vertex(
            ID: nextVertexID(),
            Location: GFP.FVector3(
                X: maxX + offset,
                Y: height,
                Z: minZ - offset
            )
        )
        let v3 = GFP.Vertex(
            ID: nextVertexID(),
            Location: GFP.FVector3(
                X: maxX + offset,
                Y: height,
                Z: maxZ + offset
            )
        )
        let v4 = GFP.Vertex(
            ID: nextVertexID(),
            Location: GFP.FVector3(
                X: minX - offset,
                Y: height,
                Z: maxZ + offset
            )
        )
        vertices.append(contentsOf: [v1, v2, v3, v4])

        let e1 = GFP.Wall(ID: nextWallID(), Vertices: [v1.ID, v2.ID])
        let e2 = GFP.Wall(ID: nextWallID(), Vertices: [v2.ID, v3.ID])
        let e3 = GFP.Wall(ID: nextWallID(), Vertices: [v3.ID, v4.ID])
        let e4 = GFP.Wall(ID: nextWallID(), Vertices: [v4.ID, v1.ID])
        walls.append(contentsOf: [e1, e2, e3, e4])

        externalWalls = [e1.ID, e2.ID, e3.ID, e4.ID]
    }

    public func export() {
        let room: GFP.Room = GFP.Room(
            ID: "RoomId\(1)",
            Type: "Room",
            Walls: internalWalls[0]
        )

        let unit: GFP.Unit = GFP.Unit(
            Location: GFP.FVector3(X: 0, Y: 0, Z: 0),
            Rotation: GFP.FRotation(Pitch: 0, Roll: 0, Yaw: 0),
            Scale: GFP.FVector3(X: 1, Y: 1, Z: 1),
            Rooms: [room]
        )

        let floor: GFP.Floor = GFP.Floor(
            Number: 1,
            Height: maxExtent.z - minExtent.z,  // by this function call Y and Z are flipped
            Location: GFP.FVector3(X: 0, Y: 0, Z: 0),
            Rotation: GFP.FRotation(Pitch: 0, Roll: 0, Yaw: 0),
            Scale: GFP.FVector3(X: 1, Y: 1, Z: 1),
            Units: [unit],
            Walls: walls,
            ExteriorWalls: externalWalls
        )

        building = GFP.Building(
            Name: "BuildingTest",
            Floors: [floor],
            Vertices: vertices,
            Scale: GFP.FVector3(X: 1, Y: 1, Z: 1)
        )

        printBuildingAsJSON(from: building!)
    }
}
