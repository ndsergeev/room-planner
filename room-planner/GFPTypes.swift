//
//  GFPTypes.swift
//  room-planner
//
//  Created by Nikita Sergeev on 28/5/2025.
//

import Foundation
import RoomPlan
import UIKit
import simd

struct GFP {
    struct FVector3: Codable {
        var X: Float
        var Y: Float
        var Z: Float
    }

    struct FRotation: Codable {
        var Pitch: Float
        var Roll: Float
        var Yaw: Float
    }

    struct Vertex: Codable {
        var ID: String
        var Location: FVector3
    }

    struct Wall: Codable {
        var ID: String
        var Vertices: [String]
    }

    struct Room: Codable {
        var ID: String
        var `Type`: String
        var Walls: [String]
    }

    struct Unit: Codable {
        var Location: FVector3
        var Rotation: FRotation
        var Scale: FVector3
        var Rooms: [Room]
    }

    struct Floor: Codable {
        var Number: Int
        //    var Entrance: Int
        var Height: Float
        //    var MarginTop: Float
        //    var MarginBottom: Float
        var Location: FVector3
        var Rotation: FRotation
        var Scale: FVector3
        var Units: [Unit]
        var Walls: [Wall]
        var ExteriorWalls: [String]
    }

    struct Building: Codable {
        var Name: String
        var Floors: [Floor]
        var Vertices: [Vertex]
        var Scale: FVector3
    }
}
