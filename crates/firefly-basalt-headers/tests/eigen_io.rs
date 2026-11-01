use firefly_basalt_headers::serialization::eigen_io::{
    ArchiveScalar, EigenMatrix, Se3Archive, Sim3Archive,
};
use nalgebra::{
    Const, DMatrix, DefaultAllocator, Dim, Dyn, OMatrix, SMatrix, allocator::Allocator,
};
use serde::{Serialize, de::DeserializeOwned};

fn fixture() -> serde_json::Value {
    serde_json::from_str(include_str!("reference/eigen_io.json")).unwrap()
}
fn bytes(text: &str) -> Vec<u8> {
    text.as_bytes()
        .as_chunks::<2>()
        .0
        .iter()
        .map(|p| u8::from_str_radix(std::str::from_utf8(p).unwrap(), 16).unwrap())
        .collect()
}
fn compare<S: ArchiveScalar + Serialize + DeserializeOwned, R: Dim, C: Dim>(
    name: &str,
    matrix: &EigenMatrix<S, R, C>,
) where
    DefaultAllocator: Allocator<R, C>,
{
    let reference = fixture();
    let reference = &reference[name];
    let expected = bytes(reference["binary"].as_str().unwrap());
    let mut actual = Vec::new();
    matrix.write_binary(&mut actual).unwrap();
    assert_eq!(actual, expected);
    let mut input = expected.as_slice();
    let restored = EigenMatrix::<S, R, C>::read_binary(&mut input).unwrap();
    assert_eq!(input, [] as [u8; 0]);
    assert_eq!(&restored, matrix);
    assert_eq!(serde_json::to_value(matrix).unwrap(), reference["json"]);
    let restored = serde_json::from_value::<EigenMatrix<S, R, C>>(reference["json"].clone());
    assert_eq!(
        restored.is_ok(),
        reference["json_load_ok"].as_bool().unwrap()
    );
    if let Ok(restored) = restored
        && !matrix.0.is_empty()
    {
        assert_eq!(&restored, matrix);
    }
    for end in 0..expected.len() {
        assert!(EigenMatrix::<S, R, C>::read_binary(&mut &expected[..end]).is_err());
    }
}
#[test]
#[cfg(target_endian = "little")]
fn original_fixed_and_dynamic_matrix_formats() {
    let values = [1., -2.5, 3.25, 4., 5.5, -6.];
    compare(
        "fixed_f64",
        &EigenMatrix(SMatrix::<f64, 2, 3>::from_row_slice(&values)),
    );
    compare(
        "fixed_f32",
        &EigenMatrix(SMatrix::<f32, 2, 3>::from_row_slice(
            &values.map(|v| v as f32),
        )),
    );
    compare(
        "fixed_i32",
        &EigenMatrix(SMatrix::<i32, 2, 2>::from_row_slice(&[-1, 2, 0, i32::MAX])),
    );
    compare(
        "dynamic_rows",
        &EigenMatrix(OMatrix::<f64, Dyn, Const<3>>::from_fn_generic(
            Dyn(2),
            Const,
            |r, c| values[r * 3 + c],
        )),
    );
    compare(
        "dynamic_cols",
        &EigenMatrix(OMatrix::<f64, Const<2>, Dyn>::from_fn_generic(
            Const,
            Dyn(3),
            |r, c| values[r * 3 + c],
        )),
    );
    compare(
        "dynamic",
        &EigenMatrix(DMatrix::<f64>::from_row_slice(2, 3, &values)),
    );
    compare("empty_dynamic", &EigenMatrix(DMatrix::<f64>::zeros(0, 3)));
}
#[test]
#[cfg(target_endian = "little")]
fn original_pose_and_scaled_quaternion_formats() {
    let reference = fixture();
    for name in ["se3", "raw_se3"] {
        let expected = bytes(reference[name]["binary"].as_str().unwrap());
        let pose: Se3Archive = serde_json::from_value(reference[name]["json"].clone()).unwrap();
        let mut actual = Vec::new();
        pose.write_binary(&mut actual).unwrap();
        assert_eq!(actual, expected);
        let restored = Se3Archive::read_binary(&mut expected.as_slice()).unwrap();
        assert_eq!(
            serde_json::to_value(restored).unwrap(),
            reference[name]["json"]
        );
        for end in 0..expected.len() {
            assert!(Se3Archive::read_binary(&mut &expected[..end]).is_err());
        }
    }
    for name in ["sim3", "raw_sim3"] {
        let expected = bytes(reference[name]["binary"].as_str().unwrap());
        let pose: Sim3Archive = serde_json::from_value(reference[name]["json"].clone()).unwrap();
        let mut actual = Vec::new();
        pose.write_binary(&mut actual).unwrap();
        assert_eq!(actual, expected);
        let restored = Sim3Archive::read_binary(&mut expected.as_slice()).unwrap();
        assert_eq!(
            serde_json::to_value(restored).unwrap(),
            reference[name]["json"]
        );
        for end in 0..expected.len() {
            assert!(Sim3Archive::read_binary(&mut &expected[..end]).is_err());
        }
    }
}
#[test]
fn matrix_shape_validation_and_original_partial_dimension_truncation() {
    assert!(serde_json::from_str::<EigenMatrix<f64, Const<2>, Const<3>>>("[1,2]").is_err());
    let matrix: EigenMatrix<f64, Dyn, Const<3>> = serde_json::from_str("[1,2,3,4,5,6,7]").unwrap();
    assert_eq!(matrix.0.shape(), (2, 3));
    assert_eq!(matrix.0[(1, 2)], 6.);
    let mut binary = 7_u64.to_ne_bytes().to_vec();
    for value in 1..=7 {
        binary.extend_from_slice(&f64::from(value).to_ne_bytes());
    }
    let mut input = binary.as_slice();
    let matrix = EigenMatrix::<f64, Dyn, Const<3>>::read_binary(&mut input).unwrap();
    assert_eq!(matrix.0.shape(), (2, 3));
    assert_eq!(input, 7_f64.to_ne_bytes());
    let mut huge = u64::MAX.to_ne_bytes().to_vec();
    huge.extend_from_slice(&2_u64.to_ne_bytes());
    assert!(EigenMatrix::<f64, Dyn, Dyn>::read_binary(&mut huge.as_slice()).is_err());
}
